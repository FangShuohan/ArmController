"""串口协议工具。当前实机使用 UartRxSyncSender：A5 + float32 deg + CRC16。

仅发送整组绝对电机角度，不生成速度曲线，不接收反馈。
旧 UARTClient / AA55 + JSON + CRC32 协议供双向软件回归使用，与当前固件不兼容。
"""
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import json
import math
import secrets
import struct
from time import monotonic, sleep
import zlib

from ArmController.validation import finite_vector

MAGIC = b"\xaa\x55"
VERSION = 1
HEADER = struct.Struct("<2sBBIH")
CRC = struct.Struct("<I")
MAX_PAYLOAD = 8192


class ProtocolError(RuntimeError):
    """协议错误、错误 ACK、重放反馈或不支持协同。"""


class MessageType(IntEnum):
    QUERY = 1
    MOVE = 2
    STOP = 3
    HELLO = 4
    ACK = 128
    FEEDBACK = 129
    ERROR = 255


def _integer(value, name, minimum=0, maximum=0xffffffff):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ProtocolError(f"{name} must be an integer in [{minimum},{maximum}]")
    return value


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


@dataclass(frozen=True)
class Frame:
    message_type: MessageType
    sequence: int
    payload: dict


def encode_frame(message_type, sequence, payload):
    """固件/模拟器也可复用相同编码器；CRC 覆盖头和载荷，不含 CRC 自身。"""
    kind = MessageType(message_type)
    _integer(sequence, "sequence", 1)
    if not isinstance(payload, dict):
        raise ProtocolError("payload must be a JSON object")
    data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > MAX_PAYLOAD:
        raise ProtocolError("payload too large")
    body = HEADER.pack(MAGIC, VERSION, kind, sequence, len(data)) + data
    return body + CRC.pack(zlib.crc32(body) & 0xffffffff)


class FrameDecoder:
    """支持半包/粘包/噪声；CRC 错误丢弃重同步，不将损坏帧交给驱动。"""
    def __init__(self):
        self.buffer = bytearray()
        self.rejected_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            offset = self.buffer.find(MAGIC)
            if offset < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(MAGIC[:1]) else b""
                break
            del self.buffer[:offset]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, seq, size = HEADER.unpack_from(self.buffer)
            if version != VERSION or kind not in MessageType._value2member_map_ or seq == 0 or size > MAX_PAYLOAD:
                del self.buffer[0]
                self.rejected_frames += 1
                continue
            total = HEADER.size + size + CRC.size
            if len(self.buffer) < total:
                # 损坏的长度字段不能永久挡住后面已完整到达的有效帧。
                next_offset = self.buffer.find(MAGIC, 2)
                if next_offset >= 0 and self._complete_valid_at(next_offset):
                    del self.buffer[:next_offset]
                    self.rejected_frames += 1
                    continue
                break
            body = bytes(self.buffer[:HEADER.size + size])
            expected = CRC.unpack_from(self.buffer, HEADER.size + size)[0]
            if zlib.crc32(body) & 0xffffffff != expected:
                del self.buffer[0]
                self.rejected_frames += 1
                continue
            del self.buffer[:total]
            try:
                payload = json.loads(body[HEADER.size:], object_pairs_hook=_unique_pairs,
                                     parse_constant=lambda value: (_ for _ in ()).throw(ProtocolError(f"invalid number {value}")))
            except (ValueError, UnicodeError) as exc:
                raise ProtocolError("invalid JSON payload") from exc
            if not isinstance(payload, dict):
                raise ProtocolError("payload must be a JSON object")
            frames.append(Frame(MessageType(kind), seq, payload))
        return frames

    def _complete_valid_at(self, offset):
        if len(self.buffer) - offset < HEADER.size:
            return False
        _, version, kind, seq, size = HEADER.unpack_from(self.buffer, offset)
        end = offset + HEADER.size + size
        if (version != VERSION or kind not in MessageType._value2member_map_ or seq == 0
                or size > MAX_PAYLOAD or len(self.buffer) < end + CRC.size):
            return False
        return zlib.crc32(self.buffer[offset:end]) & 0xffffffff == CRC.unpack_from(self.buffer, end)[0]


@dataclass(frozen=True)
class Feedback:
    motor_ids: tuple[int, ...]
    position_deg: tuple[float, ...]
    velocity_deg_s: tuple[float, ...]
    state: str
    phase: float
    motion_sequence: int
    sample_id: int
    boot_id: int
    phase_sync: bool

    @classmethod
    def from_payload(cls, data):
        try:
            ids = tuple(data["motor_ids"])
            if not ids or len(set(ids)) != len(ids):
                raise ProtocolError("motor IDs must be nonempty and unique")
            for motor_id in ids:
                _integer(motor_id, "motor_id", 1, 255)
            position = tuple(float(v) for v in finite_vector(data["position_deg"], len(ids), "position_deg"))
            velocity = tuple(float(v) for v in finite_vector(data["velocity_deg_s"], len(ids), "velocity_deg_s"))
            state = data["state"]
            phase = data["phase"]
            if state not in ("idle", "moving", "stopped", "fault"):
                raise ProtocolError("unknown MCU state")
            if isinstance(phase, bool) or not isinstance(phase, (int, float)) or not math.isfinite(phase) or not 0 <= phase <= 1:
                raise ProtocolError("phase must be a finite number in [0,1]")
            if not isinstance(data["phase_sync"], bool):
                raise ProtocolError("phase_sync must be boolean")
            return cls(ids, position, velocity, state, float(phase),
                       _integer(data["motion_sequence"], "motion_sequence"),
                       _integer(data["sample_id"], "sample_id"),
                       _integer(data["boot_id"], "boot_id"), data["phase_sync"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid feedback: {exc}") from exc


class UARTClient:
    """单线程请求/应答；stream 提供 read/write/close，可注入软件串口。

    不自动重发运动帧：ACK 超时意味着执行状态未知，应 STOP 并重新读反馈。
    固件必须按 sequence 去重，不能重复执行已接受的 MOVE。
    """
    def __init__(self, stream, *, timeout_s=0.5):
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        self.stream = stream
        self.timeout_s = float(timeout_s)
        if hasattr(stream, "timeout"):
            stream.timeout = min(0.05, self.timeout_s)
        if hasattr(stream, "write_timeout"):
            stream.write_timeout = self.timeout_s
        self.decoder = FrameDecoder()
        self.pending = deque()
        self.sequence = 0
        self.last_feedback = None
        self.session_id = secrets.token_hex(8)
        self.connected = False

    @classmethod
    def open(cls, port, *, baudrate=115200, timeout_s=0.5):
        """打开真实 UART：8 数据位、无奇偶校验、1 停止位；依赖 pyserial。"""
        _integer(baudrate, "baudrate", 1)
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        import serial
        return cls(serial.Serial(port, baudrate=baudrate, timeout=min(0.05, timeout_s),
                                 write_timeout=timeout_s), timeout_s=timeout_s)

    def _request(self, kind, payload, expected):
        if kind != MessageType.HELLO and not self.connected:
            _, hello = self._request(MessageType.HELLO, {"session_id": self.session_id}, MessageType.ACK)
            if hello.get("accepted") is not True or hello.get("session_id") != self.session_id:
                raise ProtocolError("STM32 did not acknowledge the UART session")
            self.connected = True
        if self.sequence >= 0xffffffff:
            raise ProtocolError("sequence exhausted; reconnect")
        self.sequence += 1
        seq = self.sequence
        wire = encode_frame(kind, seq, dict(payload, session_id=self.session_id))
        deadline = monotonic() + self.timeout_s
        offset = 0
        while offset < len(wire):
            if monotonic() >= deadline:
                raise TimeoutError("UART write timed out")
            count = self.stream.write(wire[offset:])
            if isinstance(count, bool) or not isinstance(count, int) or not 0 < count <= len(wire) - offset:
                raise OSError("UART write made no progress or returned invalid count")
            offset += count
        while monotonic() < deadline:
            if not self.pending:
                available = getattr(self.stream, "in_waiting", 0)
                self.pending.extend(self.decoder.feed(self.stream.read(min(max(available, 1), 4096))))
                continue
            frame = self.pending.popleft()
            if frame.sequence < seq:
                continue                       # 丢弃迟到的旧回复
            if frame.sequence != seq:
                raise ProtocolError("unexpected response sequence")
            if frame.message_type == MessageType.ERROR:
                raise ProtocolError(f"STM32 rejected request: {frame.payload}")
            if frame.payload.get("session_id") != self.session_id:
                raise ProtocolError("response belongs to a different UART session")
            if frame.message_type != expected:
                raise ProtocolError("unexpected response type")
            return seq, frame.payload
        raise TimeoutError(f"UART response timed out (sequence {seq})")

    def read_feedback(self):
        _, data = self._request(MessageType.QUERY, {}, MessageType.FEEDBACK)
        feedback = Feedback.from_payload(data)
        previous = self.last_feedback
        if previous is not None:
            if feedback.boot_id != previous.boot_id:
                raise ProtocolError("STM32 rebooted; reconnect and replan")
            if feedback.sample_id <= previous.sample_id:
                raise ProtocolError("feedback sample was replayed or reversed")
        self.last_feedback = feedback
        return feedback

    def move_segment(self, motor_ids, start_deg, target_deg, tolerance_deg, stopped_deg_s):
        """原子提交全体电机的一段绝对角度，返回被 MCU 接受的命令序号。"""
        ids = tuple(motor_ids)
        if not ids or len(set(ids)) != len(ids):
            raise ProtocolError("motor IDs must be nonempty and unique")
        for motor_id in ids:
            _integer(motor_id, "motor_id", 1, 255)
        values = [finite_vector(v, len(ids), name) for v, name in (
            (start_deg, "start_deg"), (target_deg, "target_deg"),
            (tolerance_deg, "tolerance_deg"), (stopped_deg_s, "stopped_deg_s"))]
        if any((v <= 0).any() for v in values[2:]):
            raise ProtocolError("tolerances must be positive")
        seq, data = self._request(MessageType.MOVE, {
            "motor_ids": ids, "start_deg": values[0].tolist(), "target_deg": values[1].tolist(),
            "tolerance_deg": values[2].tolist(), "stopped_deg_s": values[3].tolist(),
            "coordination": "common_phase",
        }, MessageType.ACK)
        if data.get("accepted") is not True or data.get("phase_sync") is not True:
            raise ProtocolError("STM32 did not accept common-phase execution")
        return seq

    def stop(self):
        """请求受控停止；ACK 只代表接收，停稳必须继续看反馈。"""
        _, data = self._request(MessageType.STOP, {}, MessageType.ACK)
        if data.get("accepted") is not True:
            raise ProtocolError("STM32 did not accept STOP")

    def close(self):
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# UartRxSync 固定帧单向协议。与上面的 AA55/JSON/ACK 协议互不兼容。
def crc16_modbus(payload):
    """app.cpp 的 CRC：初值 FFFF，多项式 A001，无最终异或，仅覆盖 payload。"""
    crc = 0xffff
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


class UartRxSyncSender:
    """固定头 + N 个小端 float32 deg + 小端 CRC16；纯发送接口。

    channel_motor_ids 明确指定固件通道顺序，ID 自身不进入帧。
    本类不依赖电机数量、机械臂构型或 S 算法，不读 ACK/反馈。
    当前 app.cpp 使用 header=A5，channel_motor_ids 长度为 4。
    UartRxSync 本身不规定 CRC；这里适配 app.cpp 的 decode()。
    """
    def __init__(self, stream, *, channel_motor_ids, header=b"\xa5", timeout_s=0.5):
        self.channel_motor_ids = self._ids(channel_motor_ids)
        if not isinstance(header, bytes) or not header:
            raise ValueError("header must be nonempty bytes")
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        self.stream, self.header, self.timeout_s = stream, header, float(timeout_s)
        self.payload_struct = struct.Struct("<" + "f" * len(self.channel_motor_ids))
        self.failed = False
        self.closed = False
        self.last_frame = None
        self.last_tx_frames = 0
        self.last_tx_bytes = 0
        self.last_queue_bytes = None
        if hasattr(stream, "write_timeout"):
            stream.write_timeout = self.timeout_s

    @staticmethod
    def _ids(ids):
        ids = tuple(ids)
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("motor IDs must be nonempty and unique")
        for motor_id in ids:
            _integer(motor_id, "motor_id", 1, 255)
        return ids

    @classmethod
    def open(cls, port, *, channel_motor_ids, baudrate=115200, header=b"\xa5", timeout_s=0.5):
        # 先验证参数，避免无效配置打开串口后留下资源。
        probe = cls(None, channel_motor_ids=channel_motor_ids, header=header, timeout_s=timeout_s)
        _integer(baudrate, "baudrate", 1)
        import serial
        probe.stream = serial.serial_for_url(port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1,
                                     timeout=0, write_timeout=timeout_s, xonxoff=False,
                                     rtscts=False, dsrdtr=False)
        return probe

    def quantize_angles(self, motor_ids, angles_deg):
        """验证映射和 float32 可表示性，返回保持调用者电机顺序的量化角度。"""
        ids = self._ids(motor_ids)
        if set(ids) != set(self.channel_motor_ids):
            raise ValueError("motor IDs must exactly match the configured UART channels")
        values = finite_vector(angles_deg, len(ids), "angles_deg")
        try:
            wire = self.payload_struct.pack(*values)
        except (OverflowError, struct.error) as exc:
            raise ValueError("angles must fit finite float32") from exc
        result = self.payload_struct.unpack(wire)
        if not all(math.isfinite(v) for v in result):
            raise ValueError("angles must fit finite float32")
        return result

    def encode_angles(self, motor_ids, angles_deg):
        ids = tuple(motor_ids)
        values = self.quantize_angles(ids, angles_deg)
        by_id = dict(zip(ids, values))
        payload = self.payload_struct.pack(*(by_id[i] for i in self.channel_motor_ids))
        return self.header + payload + struct.pack("<H", crc16_modbus(payload))

    def _write_frame(self, frame):
        if self.closed or self.failed:
            raise OSError("sender is closed or failed; reopen the UART before sending")
        deadline = monotonic() + self.timeout_s
        offset = 0
        try:
            while offset < len(frame):
                if monotonic() >= deadline:
                    raise TimeoutError("UART write timed out")
                count = self.stream.write(frame[offset:])
                if isinstance(count, bool) or not isinstance(count, int) or not 0 < count <= len(frame) - offset:
                    raise OSError("UART write made no progress or returned invalid count")
                offset += count
            # 等系统发送队列排空，使用同一截止时间，避免 flush() 无限等待。
            # 队列为 0 仍只是驱动报告，不是 TX 波形或板端接收确认。
            # pyserial loop:// 将同一回环队列同时计作 out_waiting/in_waiting；
            # 等它清零会等到调用者读回数据，不能按真实串口的发送队列处理。
            virtual_loop = type(self.stream).__module__ == "serial.urlhandler.protocol_loop"
            self.last_queue_bytes = None if virtual_loop else getattr(self.stream, "out_waiting", None)
            while self.last_queue_bytes is not None and self.last_queue_bytes > 0:
                if monotonic() >= deadline:
                    raise TimeoutError("UART output queue did not drain")
                sleep(min(0.001, max(0.0, deadline - monotonic())))
                self.last_queue_bytes = self.stream.out_waiting
            if monotonic() >= deadline:
                raise TimeoutError("UART write timed out")
        except (Exception, KeyboardInterrupt):
            # 部分写出时接收端帧状态未知，不自动续写或重新发送其他目标。
            self.failed = True
            raise

    def send_angles(self, motor_ids, angles_deg, *, repeat_for_s=0.0, period_s=0.05, cancel_event=None):
        """写出同一绝对目标，返回完整写出的帧数（不是接收/到位确认）。

        默认只发一次。repeat_for_s>0 时立即发第一帧，再按间隔重复。
        各帧间留空隙，供固件 IT→DMA 切换与帧重同步；不拼接突发多帧。
        Ctrl+C 或 cancel_event 仅停止后续发送，当前完整帧写完后退出。
        当前协议没有 STOP 指令，取消发送不能停止电机。
        """
        if isinstance(repeat_for_s, bool) or not math.isfinite(repeat_for_s) or repeat_for_s < 0:
            raise ValueError("repeat_for_s must be nonnegative and finite")
        if isinstance(period_s, bool) or not math.isfinite(period_s) or period_s <= 0:
            raise ValueError("period_s must be positive and finite")
        frame = self.encode_angles(motor_ids, angles_deg)
        self.last_frame = frame
        self.last_tx_frames = 0
        self.last_tx_bytes = 0
        self.last_queue_bytes = None
        started = monotonic()
        count = 0
        while True:
            if cancel_event is not None and cancel_event.is_set():
                return count
            self._write_frame(frame)
            count += 1
            self.last_tx_frames = count
            self.last_tx_bytes = count * len(frame)
            remaining = started + repeat_for_s - monotonic()
            if remaining <= period_s:
                return count
            if cancel_event is None:
                sleep(period_s)
            elif cancel_event.wait(period_s):
                return count
            if monotonic() >= started + repeat_for_s:
                return count

    def close(self):
        if not self.closed:
            self.closed = True
            self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
