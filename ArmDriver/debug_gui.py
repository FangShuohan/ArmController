"""通用 JSON 建模、虚拟预览与 UartRxSync 实机发送。运行 python -m ArmDriver。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
from threading import Event
from datetime import datetime, timedelta, timezone
from time import monotonic
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from ArmController import Capsule, IKSettings, Pose
from ArmDriver import (ArmDriver, MotorPairPreference, UartRxSyncSender,
                       model_from_dict, model_to_dict)


ROOT = Path(__file__).resolve().parent


# 兼容已有可视化调用；JSON 转换只在模型层实现。
load_description = model_from_dict
model_description = model_to_dict


def load_motor_preference(data, model):
    control = data.get("control", {})
    if not isinstance(control, dict) or set(control) - {"motor_pair_preference"}:
        raise ValueError("control 只支持 motor_pair_preference")
    options = control.get("motor_pair_preference")
    if options is None:
        return None
    if (not isinstance(options, dict) or "motor_ids" not in options or
            set(options) - {"motor_ids", "tolerance_deg"}):
        raise ValueError("motor_pair_preference 必须含 motor_ids，可选 tolerance_deg")
    preference = MotorPairPreference(**options)
    preference.indices(model.motor_ids)
    return preference


def load_configuration(data):
    model = model_from_dict(data)
    if "model" in data and set(data) - {"model", "uart", "control"}:
        raise ValueError("配置顶层只支持 model、uart 和 control")
    load_motor_preference(data, model)
    uart = data.get("uart", {"channel_motor_ids": list(model.motor_ids), "baudrate": 115200})
    if not isinstance(uart, dict) or set(uart) != {"channel_motor_ids", "baudrate"}:
        raise ValueError("uart 必须包含 channel_motor_ids 和 baudrate")
    encoder = UartRxSyncSender(None, channel_motor_ids=uart["channel_motor_ids"])
    encoder.quantize_angles(model.motor_ids, model.zero_deg)
    baud = uart["baudrate"]
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise ValueError("baudrate 必须是正整数")
    return model, dict(channel_motor_ids=list(encoder.channel_motor_ids), baudrate=baud)


def user_joints(model, q):
    return [float(np.rad2deg(v) if j.joint_type == "hinge" else v) for j,v in zip(model.joints,q)]


def initial_joints(model):
    """零位不在行程内时采用行程内参考姿态，界面明确显示关节值。"""
    q = np.clip(np.zeros(model.dof),model.collision.lower,model.collision.upper)
    model.joints_to_motor(q)
    return q


def geometry(model, q):
    q = model.collision.validate_joint_variables(q)
    transforms = model.kinematics.prefix_transforms(q)
    points, axes = [], []
    for i,j in enumerate(model.joints):
        t = transforms[i+1] if j.joint_type == "slide" else transforms[i]
        points.append(t[:3,:3] @ np.asarray(j.axis_point_m) + t[:3,3])
        axes.append(t[:3,:3] @ np.asarray(j.axis_xyz))
    pose = Pose.from_matrix(transforms[-1] @ model.kinematics.zero_transform)
    capsules = []
    for c in model.collision.capsules:
        t = transforms[c.joint_count]
        capsules.append(Capsule(c.name,c.joint_count,t[:3,:3]@c.start+t[:3,3],
                                t[:3,:3]@c.end+t[:3,3],c.radius))
    return np.array(points),np.array(axes),pose,tuple(capsules)


def draw_model(ax, model, q, target_pose=None, limits=None):
    """复用三维图元；仅模型改变时重建场景。返回固定视野范围。"""
    renderer = getattr(ax,"_arm_renderer",None)
    if renderer is None or renderer.model is not model:
        renderer = ModelRenderer(ax,model)
        ax._arm_renderer = renderer
    return renderer.update(q,target_pose,limits)


class ModelRenderer:
    def __init__(self,ax,model):
        self.ax,self.model=ax,model
        self.background=None
        ax.clear()
        ax.set_box_aspect((1,1,1))
        ax.set_xlabel("X / m"); ax.set_ylabel("Y / m"); ax.set_zlabel("Z / m")
        ax.set_title("Virtual model | joint axes + envelope centerlines",fontsize=10)
        self.links=Line3DCollection([],linewidths=7,alpha=.65,colors="#397fbb")
        self.axes=Line3DCollection([],linewidths=1.8,colors="#ad7a16")
        self.frames=Line3DCollection([],linewidths=1.8)
        for artist in (self.links,self.axes,self.frames): ax.add_collection3d(artist,autolim=False)
        self.joints,=ax.plot([],[],[],"o",color="#18324d",markersize=5)
        self.tool,=ax.plot([],[],[],"o",color="#16a085",markersize=7)
        self.target,=ax.plot([],[],[],"x",color="#d95d39",markersize=9)
        self.labels=[ax.text(0,0,0,f" J{i+1}",fontsize=9) for i in range(model.dof)]
        self.artists=[self.links,self.axes,self.frames,self.joints,self.tool,self.target,*self.labels]
        self.animated=False
        self.pose=None

    def enable_blit(self):
        self.animated=True
        for artist in self.artists: artist.set_animated(True)

    def capture(self):
        # 视角、缩放或窗口尺寸改变后，完整绘制事件重新建立背景。
        self.background=self.ax.figure.canvas.copy_from_bbox(self.ax.bbox)
        self.paint()

    def paint(self):
        for artist in self.artists:
            if not artist.get_visible(): continue
            if hasattr(artist,"do_3d_projection"): artist.do_3d_projection()
            self.ax.draw_artist(artist)

    def blit(self):
        canvas=self.ax.figure.canvas
        if self.background is None:
            canvas.draw_idle()
        else:
            canvas.restore_region(self.background)
            self.paint()
            canvas.blit(self.ax.bbox)

    def update(self,q,target_pose,limits):
        ax,model=self.ax,self.model
        points,axes,pose,capsules = geometry(model,q)
        self.pose=pose
        length=max(.04,float(np.ptp(np.vstack((points,pose.position)),axis=0).max())*.13)
        self.links.set_segments([np.vstack((c.start,c.end)) for c in capsules])
        # 一次更新所有轴线，不再逐帧创建 quiver/scatter 对象。
        arrows=[]
        for p,a in zip(points,axes):
            end=p+length*a
            across=np.cross(a,[1,0,0] if abs(a[0])<.9 else [0,1,0])
            across/=np.linalg.norm(across)
            arrows.extend((np.vstack((p,end)),
                           np.vstack((end-.2*length*a+.08*length*across,end,
                                      end-.2*length*a-.08*length*across))))
        self.axes.set_segments(arrows)
        segments,colors=[],[]
        frames=[(np.zeros(3),np.eye(3),length),(pose.position,pose.as_matrix()[:3,:3],length)]
        if target_pose is not None:
            frames.append((target_pose.position,target_pose.as_matrix()[:3,:3],length*.65))
        for origin,rotation,size in frames:
            for i,color in enumerate(("#d84c4c","#269969","#487ddb")):
                segments.append(np.vstack((origin,origin+size*rotation[:,i])))
                colors.append(color)
        self.frames.set_segments(segments); self.frames.set_color(colors)
        self.joints.set_data_3d(*points.T)
        self.tool.set_data_3d(*pose.position[:,None])
        self.target.set_visible(target_pose is not None)
        if target_pose is not None: self.target.set_data_3d(*target_pose.position[:,None])
        for label,point in zip(self.labels,points): label.set_position_3d(point)
        if limits is None:
            cloud=np.vstack((np.zeros(3),points,pose.position,*[np.vstack((c.start,c.end)) for c in capsules]))
            if target_pose is not None: cloud=np.vstack((cloud,target_pose.position))
            center=(cloud.min(axis=0)+cloud.max(axis=0))/2
            radius=max(.2,float(np.ptp(cloud,axis=0).max())*.65)
            limits=tuple((c-radius,c+radius) for c in center)
            ax.set_xlim(*limits[0]); ax.set_ylim(*limits[1]); ax.set_zlim(*limits[2])
            self.background=None
        return limits


class DebugApp:
    def __init__(self, root, description=None):
        self.root=root
        root.title("ArmDriver · 建模、预览与实机发送")
        root.geometry("1380x880")
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.future=None
        self.future_kind="preview"
        self.serial_pool=ThreadPoolExecutor(max_workers=1)
        self.serial_future=None
        self.serial_action=None
        self.sender=None
        self.serial_details="尚未连接或下发"
        self.cancel_send=Event()
        self.pending_target=None
        self.pending_snapshot=None
        self.last_sent_motor_deg=None
        self.sending_target=None
        self.discard_result=False
        self.closed=False
        self.animation=None
        self.target=None
        self.limits=None
        self.last_readout=0.0
        self.status=tk.StringVar(value="加载模型后可进行正逆运动学预览")
        self.duration=tk.StringVar(value="2.0")
        self.position_only=tk.BooleanVar(value=False)
        self.prefer_symmetric=tk.BooleanVar(value=False)
        self.configured_preference=None
        self.constraint_hint=tk.StringVar()
        self.path_check=tk.BooleanVar(value=True)
        self.model=None
        self.path=ROOT/"arm_config.json"
        self._build_ui()
        self.editor.insert("1.0",json.dumps(description if description is not None else json.loads(self.path.read_text(encoding="utf-8-sig")),ensure_ascii=False,indent=2))
        self.apply_model()
        if self.model is None:
            self.pool.shutdown(wait=False,cancel_futures=True)
            self.serial_pool.shutdown(wait=False,cancel_futures=True)
            message=self.status.get()
            root.destroy()
            raise ValueError(message)
        root.protocol("WM_DELETE_WINDOW",self.close)
        self.timer=root.after(16,self.tick)

    def _build_ui(self):
        top=ttk.Frame(self.root,padding=8); top.pack(fill="x")
        ttk.Label(top,text="ArmDriver",font=("Microsoft YaHei UI",15,"bold")).pack(side="left")
        ttk.Label(top,text="   虚拟预览 + 单向 UART · 无实机反馈",foreground="#a35d13").pack(side="left")
        for text,command in (("打开参数",self.open_config),("保存参数",self.save_config),
                             ("导出当前配置",self.export_model),("使用说明",self.open_guide)):
            ttk.Button(top,text=text,command=command).pack(side="right",padx=3)
        pane=ttk.Panedwindow(self.root,orient="horizontal"); pane.pack(fill="both",expand=True,padx=8)
        left=ttk.Frame(pane,width=410); pane.add(left,weight=0)
        right=ttk.Frame(pane); pane.add(right,weight=1)
        notebook=ttk.Notebook(left); notebook.pack(fill="both",expand=True)
        target_tab=ttk.Frame(notebook); notebook.add(target_tab,text="目标与关节")
        scroller=tk.Canvas(target_tab,highlightthickness=0,width=420)
        target_scroll=ttk.Scrollbar(target_tab,orient="vertical",command=scroller.yview)
        scroller.configure(yscrollcommand=target_scroll.set)
        target_scroll.pack(side="right",fill="y"); scroller.pack(side="left",fill="both",expand=True)
        targets=ttk.Frame(scroller,padding=10)
        target_window=scroller.create_window((0,0),window=targets,anchor="nw")
        targets.bind("<Configure>",lambda event:scroller.configure(scrollregion=scroller.bbox("all")))
        scroller.bind("<Configure>",lambda event:scroller.itemconfigure(target_window,width=event.width))
        config=ttk.Frame(notebook,padding=8); notebook.add(config,text="模型参数")
        ttk.Label(config,text="编辑 JSON 后点击应用；支持任意关节数的通用模型。",wraplength=390).pack(anchor="w")
        box=ttk.Frame(config); box.pack(fill="both",expand=True)
        self.editor=tk.Text(box,wrap="none",width=48,undo=True,font=("Consolas",10))
        scroll=ttk.Scrollbar(box,orient="vertical",command=self.editor.yview)
        self.editor.configure(yscrollcommand=scroll.set)
        self.editor.pack(side="left",fill="both",expand=True); scroll.pack(side="right",fill="y")
        ttk.Button(config,text="校验并应用参数",command=self.apply_model).pack(fill="x",pady=8)
        ttk.Label(targets,text="工具目标位置 / m",font=("Microsoft YaHei UI",10,"bold")).pack(anchor="w")
        self.xyz=self._vector_entries(targets,("X","Y","Z"))
        ttk.Label(targets,text="工具姿态 / deg：固定基座轴 xyz 欧拉角").pack(anchor="w",pady=(8,0))
        self.rpy_entries=[]
        self.rpy=self._vector_entries(targets,("Rx","Ry","Rz"),entries=self.rpy_entries)
        ttk.Checkbutton(targets,text="只约束位置，忽略目标姿态",variable=self.position_only).pack(anchor="w")
        self.preference_check=ttk.Checkbutton(targets,text="逆解优先电机等角/反角（软优先）",
                                              variable=self.prefer_symmetric)
        self.preference_check.pack(anchor="w")
        ttk.Label(targets,text="保持原目标，必要时保留组合解；关节目标不受影响。",
                  wraplength=375).pack(anchor="w",pady=3)
        ttk.Label(targets,textvariable=self.constraint_hint,wraplength=375,foreground="#936324").pack(anchor="w",pady=3)
        ttk.Button(targets,text="从当前虚拟位姿填入目标",command=self.use_current_pose).pack(fill="x",pady=3)
        ttk.Button(targets,text="求逆解并预览",command=self.solve_pose).pack(fill="x",pady=6)
        ttk.Separator(targets).pack(fill="x",pady=8)
        ttk.Label(targets,text="逻辑关节目标：旋转 deg / 伸缩 m").pack(anchor="w")
        ttk.Label(targets,text="这里填写机构关节角度。两台末端电机的角度请看下方换算结果。",wraplength=375).pack(anchor="w",pady=3)
        self.joint_frame=ttk.Frame(targets); self.joint_frame.pack(fill="x")
        self.joint_motor_text=tk.StringVar()
        ttk.Label(targets,text="物理电机目标 / deg（由上方关节输入换算，未发送）",
                  wraplength=375).pack(anchor="w",pady=(8,2))
        ttk.Label(targets,textvariable=self.joint_motor_text,font=("Consolas",10),
                  wraplength=375).pack(anchor="w")
        ttk.Button(targets,text="按关节目标预览",command=self.preview_joints).pack(fill="x",pady=6)
        row=ttk.Frame(targets); row.pack(fill="x")
        ttk.Label(row,text="动画时间 / s").pack(side="left")
        ttk.Entry(row,textvariable=self.duration,width=7).pack(side="left",padx=6)
        ttk.Button(row,text="暂停预览",command=self.pause).pack(side="left",padx=3)
        ttk.Button(row,text="参考姿态",command=self.reset).pack(side="left")
        ttk.Checkbutton(targets,text="预览前检查理想关节直线路径",variable=self.path_check).pack(anchor="w",pady=4)
        ttk.Label(targets,text="动画是几何插值，不模拟下位机 S 曲线或实际电机跟踪。",wraplength=375,foreground="#936324").pack(anchor="w")
        ttk.Separator(targets).pack(fill="x",pady=10)
        ttk.Label(targets,text="从已知电机角度恢复虚拟姿态 / deg").pack(anchor="w")
        self.motor_input=ttk.Entry(targets); self.motor_input.pack(fill="x",pady=4)
        ttk.Button(targets,text="应用输入角度（模型 ID 顺序）",command=self.apply_motor).pack(fill="x")
        ttk.Separator(targets).pack(fill="x",pady=10)
        ttk.Label(targets,text="UartRxSync 帧预览：通道 motor_id 顺序").pack(anchor="w")
        self.channel_input=ttk.Entry(targets); self.channel_input.pack(fill="x",pady=4)
        ttk.Button(targets,text="复制当前虚拟姿态的发送帧",command=self.copy_frame).pack(fill="x")
        ttk.Label(targets,text="仅复制虚拟帧；实机发送请切换“实机串口”页。",wraplength=375).pack(anchor="w",pady=4)
        self.channel_input.configure(state="readonly")
        self._build_serial_tab(notebook)
        for var in (*self.xyz,*self.rpy,self.path_check,self.prefer_symmetric):
            var.trace_add("write",self._invalidate_pending)
        self.position_only.trace_add("write",self._update_pose_mode)
        self._update_pose_mode()
        self.figure=Figure(figsize=(8,6),dpi=100)
        self.ax=self.figure.add_subplot(111,projection="3d"); self.ax.view_init(22,-65)
        self.canvas=FigureCanvasTkAgg(self.figure,master=right)
        self.canvas.mpl_connect("draw_event",self._on_draw)
        self.canvas.get_tk_widget().pack(fill="both",expand=True)
        toolbar=NavigationToolbar2Tk(self.canvas,right,pack_toolbar=False); toolbar.pack(fill="x")
        ttk.Button(toolbar,text="重置视野",command=self.fit_view).pack(side="right")
        self.readout=tk.Text(right,height=8,wrap="word",font=("Consolas",10),state="disabled")
        self.readout.pack(fill="x",pady=4)
        ttk.Label(self.root,textvariable=self.status,wraplength=1350,padding=8).pack(fill="x")

    def _build_serial_tab(self,notebook):
        page=ttk.Frame(notebook); notebook.add(page,text="实机串口")
        scroller=tk.Canvas(page,highlightthickness=0,width=420)
        scrollbar=ttk.Scrollbar(page,orient="vertical",command=scroller.yview)
        scroller.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right",fill="y"); scroller.pack(side="left",fill="both",expand=True)
        tab=ttk.Frame(scroller,padding=10)
        content=scroller.create_window((0,0),window=tab,anchor="nw")
        tab.bind("<Configure>",lambda event:scroller.configure(scrollregion=scroller.bbox("all")))
        scroller.bind("<Configure>",lambda event:scroller.itemconfigure(content,width=event.width))
        self.port=tk.StringVar()
        self.physical_current=tk.StringVar()
        self.reuse_sent_target=tk.BooleanVar(value=False)
        self.start_hint=tk.StringVar()
        self.serial_config=tk.StringVar()
        self.serial_state=tk.StringVar(value="未连接；连接不会发送运动指令")
        self.send_report=tk.StringVar(value="尚无待发目标")
        self.transmit_state=tk.StringVar(value="发送记录：本次连接尚未下发")
        self.repeat_for=tk.StringVar(value="0.3")
        self.send_period=tk.StringVar(value="0.05")
        ttk.Label(tab,text="端口（如 COM5；软件测试可填 loop://）").pack(anchor="w")
        self.port_box=ttk.Combobox(tab,textvariable=self.port); self.port_box.pack(fill="x",pady=4)
        row=ttk.Frame(tab); row.pack(fill="x")
        for label,fn in (("刷新端口",self.refresh_ports),("连接",self.connect_serial),("断开",self.disconnect_serial),("发送详情",self.show_serial_details)):
            ttk.Button(row,text=label,command=fn).pack(side="left",padx=3)
        ttk.Button(tab,text="USB-TTL 短接检测（先断开机械臂）",command=self.probe_serial_loopback).pack(fill="x",pady=4)
        ttk.Label(tab,textvariable=self.serial_state,wraplength=380).pack(anchor="w",pady=6)
        ttk.Label(tab,textvariable=self.transmit_state,wraplength=380,
                  foreground="#a35d13").pack(anchor="w")
        ttk.Label(tab,textvariable=self.serial_config,wraplength=380).pack(anchor="w",pady=6)
        ttk.Separator(tab).pack(fill="x",pady=8)
        ttk.Label(tab,text="初始／校正用实际电机绝对角度 / deg",font=("Microsoft YaHei UI",10,"bold")).pack(anchor="w")
        ttk.Entry(tab,textvariable=self.physical_current).pack(fill="x",pady=4)
        self.physical_current.trace_add("write",self._invalidate_pending)
        ttk.Label(tab,text="按上面模型 ID 顺序填写，以空格或逗号分隔。填写值优先用于本次起点；不是目标，也不会加到目标角度上。",wraplength=380).pack(anchor="w")
        ttk.Checkbutton(tab,text="连续调试：沿用上次已发送目标作为估计起点",
                        variable=self.reuse_sent_target).pack(anchor="w",pady=4)
        self.reuse_sent_target.trace_add("write",self._update_start_hint)
        ttk.Label(tab,textvariable=self.start_hint,wraplength=380,
                  foreground="#936324").pack(anchor="w")
        self._update_start_hint()
        ttk.Label(tab,text="目标沿用“目标与关节”页输入：").pack(anchor="w",pady=(10,4))
        ttk.Button(tab,text="计算关节待发目标",command=lambda:self.prepare_physical("joints")).pack(fill="x",pady=3)
        ttk.Button(tab,text="计算位姿待发目标",command=lambda:self.prepare_physical("pose")).pack(fill="x",pady=3)
        ttk.Button(tab,text="预览待发目标（虚拟，不发送）",command=self.preview_pending_target).pack(fill="x",pady=3)
        ttk.Label(tab,textvariable=self.send_report,wraplength=380,font=("Consolas",10)).pack(anchor="w",pady=8)
        row=ttk.Frame(tab); row.pack(fill="x")
        for text,var in (("重复时长 / s",self.repeat_for),("间隔 / s",self.send_period)):
            ttk.Label(row,text=text).pack(side="left")
            ttk.Entry(row,textvariable=var,width=6).pack(side="left",padx=4)
        self.send_button=ttk.Button(tab,text="下发已计算目标",command=self.send_physical,state="disabled")
        self.send_button.pack(fill="x",pady=8)
        ttk.Button(tab,text="停止重复下发（不能停止电机）",command=self.cancel_send.set).pack(fill="x")
        ttk.Label(tab,text="只发送整组绝对电机角度。重复的是同一个目标；没有速度规划、ACK 或到位回传。关闭窗口、断开串口也不是电机停止命令。",foreground="#936324",wraplength=380).pack(anchor="w",pady=8)

    def _invalidate_pending(self,*args):
        self.pending_target=None
        self.pending_snapshot=None
        if hasattr(self,"send_button"):
            self.send_button.configure(state="disabled")
            self.send_report.set("尚无待发目标；输入变化后需重新计算")

    def _update_start_hint(self,*args):
        self._invalidate_pending()
        if not self.reuse_sent_target.get():
            text="手动模式：每次填写实际角度。虚拟预览不作为实机反馈。"
        elif self.last_sent_motor_deg is None:
            text="估计模式：首次请填写实际角度。写出目标后自动记住；板子复位或机构被移动后需重新校正。"
        else:
            text=(f"上次写出的目标 / deg: {np.round(self.last_sent_motor_deg,6).tolist()}。"
                  "输入框留空时沿用它；这是估计值，未确认到位。运动中增量与路径检查仅基于此估计。")
        self.start_hint.set(text)

    def _reset_physical_start(self):
        self.last_sent_motor_deg=None
        self.sending_target=None
        self.physical_current.set("")
        self._update_start_hint()

    def _physical_start(self):
        text=self.physical_current.get().strip()
        if text:
            current=tuple(float(v) for v in text.replace(","," ").split())
            source="手动填写的实际角度"
        elif self.reuse_sent_target.get() and self.last_sent_motor_deg is not None:
            current=self.last_sent_motor_deg
            source="上次已发送目标（估计起点，未确认到位）"
        else:
            raise ValueError("请先填写全部已确认的实际电机角度；估计模式首次也需要实机起点")
        if len(current)!=self.model.dof:
            raise ValueError("请先填写全部已确认的实际电机角度")
        return current,source

    def _input_snapshot(self):
        return (id(self.model),tuple(v.get() for v in self.joint_vars),
                tuple(v.get() for v in self.xyz+self.rpy),self.position_only.get(),self.prefer_symmetric.get(),
                self.path_check.get(),self.physical_current.get(),self.reuse_sent_target.get(),
                self.last_sent_motor_deg,self.editor.get("1.0","end"))

    def prepare_physical(self,kind):
        if not self._idle(): return
        self._invalidate_pending()
        try:
            if json.loads(self.editor.get("1.0","end")) != self.applied_description:
                raise ValueError("JSON 已编辑，请先断开串口并校验应用参数")
            current,source=self._physical_start()
            model=self.model
            start=model.motor_to_joints(current)
            model.collision.require_valid(start)
            goal=position=quat=None
            if kind=="joints":
                goal=[float(v.get()) for v in self.joint_vars]
                arm=ArmDriver(model)
            elif kind=="pose":
                position,quat,settings=self._pose_goal()
                arm=self._pose_driver(settings)
            else:
                raise ValueError("目标类型必须为 joints 或 pose")
            encoder=UartRxSyncSender(None,channel_motor_ids=self.uart_settings["channel_motor_ids"])
            snapshot,check=self._input_snapshot(),self.path_check.get()
            def calculate():
                target=(arm.solve_target_joints(goal,current_motor_deg=current) if kind=="joints" else
                        arm.solve_target_pose(position,quat,current_motor_deg=current))
                quantized=encoder.quantize_angles(target.motor_ids,target.target_deg)
                end=model.motor_to_joints(quantized)
                model.collision.require_valid(end)
                if check: model.collision.require_edge(start,end)
                pose=model.kinematics.forward(end)
                angles=Rotation.from_quat(pose.quaternion_xyzw).as_euler("xyz",degrees=True)
                report=(f"起点来源: {source}\n"
                        f"逻辑目标 deg/m: {np.round(user_joints(model,end),5).tolist()}\n"
                        f"工具结果 Rx/Ry/Rz deg: {np.round(angles,5).tolist()}\n"
                        f"电机 ID: {target.motor_ids}\n目标 deg: {np.round(quantized,6).tolist()}\n"
                        f"增量 deg: {np.round(np.array(quantized)-current,6).tolist()}\n"
                        f"帧: {encoder.encode_angles(target.motor_ids,quantized).hex(' ')}")
                if arm.motor_pair_preference is not None:
                    relation=arm.motor_pair_preference.relation(target.motor_ids,quantized)
                    labels={"equal":"等角偏好已满足", "opposite":"反角偏好已满足",
                            "combined":"保留组合解：未找到符合原任务与校验的对称解"}
                    report += f"\n电机 {arm.motor_pair_preference.motor_ids}: {labels[relation]}"
                return target,snapshot,report
            self.animation=None
            self.future_kind="physical"
            self.future=self.pool.submit(calculate)
            self.status.set(f"正在根据{source}计算待发目标；尚未发送。")
        except Exception as exc: self._error(exc)

    def preview_pending_target(self):
        if not self._idle(): return
        try:
            if self.pending_target is None or self.pending_snapshot!=self._input_snapshot():
                raise ValueError("请先计算有效的待发目标")
            duration=self._animation_duration()
            target=self.pending_target
            encoder=UartRxSyncSender(None,channel_motor_ids=self.uart_settings["channel_motor_ids"])
            angles=encoder.quantize_angles(target.motor_ids,target.target_deg)
            goal=self.model.motor_to_joints(angles)
            self.q=self.model.motor_to_joints(target.current_motor_deg)
            self.target=self.model.kinematics.forward(goal)
            self.animation=(self.q.copy(),goal,monotonic(),duration)
            self.limits=None
            self.redraw()
            self.status.set("正在虚拟预览待发目标；未发送，不代表实机位置或固件运动。")
        except Exception as exc: self._error(exc)

    def _serial_submit(self,action,work):
        self.serial_action=action
        self.serial_future=self.serial_pool.submit(work)
        self.status.set("串口任务："+action)

    def refresh_ports(self):
        if not self._idle(): return
        def discover():
            from serial.tools import list_ports
            return [p.device for p in list_ports.comports()]
        self._serial_submit("ports",discover)

    def connect_serial(self):
        if not self._idle(): return
        if self.sender is not None:
            self.status.set("串口已连接；更换端口请先断开。")
            return
        port=self.port.get().strip()
        if not port:
            self._error(ValueError("请填写串口端口")); return
        settings=dict(self.uart_settings)
        self._serial_submit("connect",lambda:UartRxSyncSender.open(port,**settings))

    def disconnect_serial(self):
        if not self._idle(): return
        self._invalidate_pending()
        self._reset_physical_start()
        if self.sender is None: return
        sender=self.sender
        self.sender=None
        self._serial_submit("disconnect",sender.close)

    def probe_serial_loopback(self):
        """仅用户手动检测时读取转换器回环，不读取/模拟下位机反馈。"""
        if not self._idle(): return
        if self.sender is None:
            self._error(ValueError("请先连接要检测的真实串口")); return
        if not messagebox.askokcancel("USB-TTL 本体短接检测",
                "先将 USB-TTL 与机械臂的接线全部断开，仅短接转换器的 TX 和 RX。\n"
                "本次会发送 ASCII 测试数据并读取回环，不发送机械臂角度帧。\n"
                "确认已按上述接线后继续。",parent=self.root):
            return
        sender=self.sender
        self._invalidate_pending(); self._reset_physical_start()
        self.serial_details=self._serial_detail_text(sender)
        self.cancel_send.clear()
        self.transmit_state.set(f"正在检测 {sender.stream.port} 的 TX/RX 短接回环")
        def probe():
            stream=sender.stream
            payload=b"ArmDriver UART loopback\r\n"*64  # 无 A5，不是运动指令
            previous_timeout=stream.timeout
            try:
                stream.timeout=0
                stream.reset_input_buffer()
                written=stream.write(payload)
                received=bytearray()
                deadline=monotonic()+2.0
                while len(received)<len(payload) and monotonic()<deadline and not self.cancel_send.is_set():
                    received.extend(stream.read(len(payload)-len(received)))
                    if len(received)<len(payload): self.cancel_send.wait(.005)
                return written,bytes(received),payload
            finally:
                stream.timeout=previous_timeout
        self._serial_submit("probe",probe)

    def send_physical(self):
        if not self._idle(): return
        try:
            if self.sender is None: raise ValueError("请先连接串口")
            if self.pending_target is None or self.pending_snapshot!=self._input_snapshot():
                self._invalidate_pending()
                raise ValueError("请重新计算待发目标")
            repeat,period=float(self.repeat_for.get()),float(self.send_period.get())
            if not np.isfinite(repeat) or repeat<0 or not np.isfinite(period) or period<=0:
                raise ValueError("重复时长必须非负，间隔必须大于 0，且都为有限数")
            sender,arm,target=self.sender,self.arm,self.pending_target
            self._invalidate_pending()
            self.physical_current.set("")
            self.sending_target=target
            self.cancel_send.clear()
            self.send_report.set("正在下发；完成后记录目标，实机到位未确认")
            self.transmit_state.set(f"发送记录：正在向 {sender.stream.port} 写出整组目标")
            self.serial_details=self._serial_detail_text(sender,target)
            self._serial_submit("send",lambda:arm.send_target(target,sender,repeat_for_s=repeat,
                                                             period_s=period,cancel_event=self.cancel_send))
        except Exception as exc: self._error(exc)

    def _poll_serial(self):
        if self.serial_future is None or not self.serial_future.done(): return
        future,action=self.serial_future,self.serial_action
        self.serial_future=None; self.serial_action=None
        try:
            result=future.result()
            if action=="connect":
                self.sender=result
                self.serial_details=self._serial_detail_text(result)
                self.transmit_state.set("发送记录：本次连接尚未下发")
                self.port_box.configure(state="disabled")
                self.serial_state.set(f"已连接 {result.stream.port}；仅端口打开，未确认下位机接收")
                if self.pending_target is not None: self.send_button.configure(state="normal")
            elif action=="disconnect":
                self.port_box.configure(state="normal")
                self.serial_state.set("已断开；没有发送电机停止指令")
            elif action=="ports":
                self.port_box.configure(values=result)
                self.status.set("可用端口："+(", ".join(result) or "未发现；仍可手动填写"))
                return
            elif action=="probe":
                written,received,payload=result
                passed=written==len(payload) and received==payload
                stream=self.sender.stream
                virtual="://" in str(stream.port)
                kind="软件回环" if virtual else "USB-TTL 短接回环"
                message=(f"{stream.port} {kind} {'PASS' if passed else 'FAIL'}；"
                         f"写入 {written} / 回收 {len(received)} 字节")
                self.serial_details += (f"\n{message}\n预期字节数: {len(payload)}"
                                        f"\n接收前64字节: {received[:64].hex(' ')}"
                                        "\n本检测不证明 STM32 接收或机械臂到位。")
                self.send_report.set(message)
                self.transmit_state.set(message)
            elif action=="send":
                sender=self.sender
                self.last_sent_motor_deg=(sender.quantize_angles(self.sending_target.motor_ids,
                                          self.sending_target.target_deg) if result>0 else None)
                self.sending_target=None
                self._update_start_hint()
                queue=sender.last_queue_bytes
                queue_text=str(queue) if queue is not None else "不适用（当前后端无系统发送队列）"
                self.serial_details += (f"\n完成时间: {datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')}"
                                        f"\n完整帧数: {sender.last_tx_frames}"
                                        f"\n完整帧字节数: {sender.last_tx_bytes}"
                                        f"\n系统剩余发送队列字节: {queue_text}"
                                        "\n以上是主机/驱动记录，未验证 TX 引脚或下位机接收。")
                message=(f"{sender.stream.port} 主机已完整写出 {result} 帧 / {sender.last_tx_bytes} 字节；"
                         "接收/到位未确认")
                self.send_report.set(message)
                self.transmit_state.set("发送记录："+message)
            self.status.set(self.send_report.get() if action in ("send","probe") else self.serial_state.get())
        except Exception as exc:
            self._invalidate_pending()
            self._reset_physical_start()
            if action in ("send","probe"):
                self.serial_details += "\n发送失败: "+str(exc)
                self.transmit_state.set("发送记录：发送失败，接收状态未知；"+str(exc))
            if action in ("send","probe") and self.sender is not None:
                connection,self.sender=self.sender,None
                self._serial_submit("disconnect",connection.close)
                self.serial_state.set("发送失败；必须重新连接并确认实际角度")
            self._error(exc)

    def _serial_detail_text(self,sender,target=None):
        stream=sender.stream
        text=(f"记录时间: {datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')}\n"
              f"实际打开端口: {stream.port}\n串口类型: {type(stream).__module__}.{type(stream).__name__}\n"
              f"配置: {stream.baudrate} baud / {stream.bytesize}{stream.parity}{stream.stopbits}\n"
              f"流控: xonxoff={stream.xonxoff}, rtscts={stream.rtscts}, dsrdtr={stream.dsrdtr}\n"
              f"串口通道 ID: {sender.channel_motor_ids}")
        if target is not None:
            frame=sender.encode_angles(target.motor_ids,target.target_deg)
            text += (f"\n模型 ID: {target.motor_ids}\n目标 deg: {target.target_deg}"
                     f"\n每帧字节数: {len(frame)}\n目标二进制帧（以下仅为十六进制显示）: {frame.hex(' ')}"
                     f"\n重复时长/间隔 s: {self.repeat_for.get()} / {self.send_period.get()}")
        return text

    def show_serial_details(self):
        window=tk.Toplevel(self.root); window.title("串口发送详情"); window.geometry("800x480")
        text=tk.Text(window,wrap="word",font=("Consolas",10))
        text.pack(fill="both",expand=True)
        text.insert("1.0",self.serial_details); text.configure(state="disabled")
        ttk.Button(window,text="复制记录",command=lambda:(self.root.clipboard_clear(),
                    self.root.clipboard_append(self.serial_details))).pack(pady=6)

    @staticmethod
    def _vector_entries(parent,labels,entries=None):
        frame=ttk.Frame(parent); frame.pack(fill="x",pady=3)
        variables=[]
        for label in labels:
            ttk.Label(frame,text=label).pack(side="left")
            variable=tk.StringVar(value="0")
            entry=ttk.Entry(frame,textvariable=variable,width=9)
            entry.pack(side="left",padx=(2,8))
            if entries is not None: entries.append(entry)
            variables.append(variable)
        return variables

    def _idle(self):
        if self.serial_future is not None:
            self.status.set("串口任务进行中，请等待完成或停止重复下发。")
            return False
        if self.future is not None:
            self.status.set("正在计算，请等待结果。")
            return False
        return True

    def _error(self,exc):
        self.status.set("未应用："+str(exc))

    def apply_model(self):
        if not self._idle(): return
        if self.sender is not None:
            self.status.set("修改模型前请先断开串口。")
            return
        try:
            model,uart=load_configuration(json.loads(self.editor.get("1.0","end")))
            q=initial_joints(model)
        except Exception as exc:
            self._error(exc); return
        self._invalidate_pending()
        self._reset_physical_start()
        self.applied_description=json.loads(self.editor.get("1.0","end"))
        self.model=model; self.uart_settings=uart; self.arm=ArmDriver(model)
        self.configured_preference=load_motor_preference(self.applied_description,model)
        self.prefer_symmetric.set(self.configured_preference is not None)
        pair=self.configured_preference.motor_ids if self.configured_preference else "未配置"
        self.preference_check.configure(text=f"逆解优先电机 {pair} 等角/反角（软优先）",
                                        state="normal" if self.configured_preference else "disabled")
        self.q=q; self.target=None; self.animation=None; self.limits=None
        for child in self.joint_frame.winfo_children(): child.destroy()
        self.joint_vars=[]
        for i,j in enumerate(model.joints):
            row=ttk.Frame(self.joint_frame); row.pack(fill="x",pady=2)
            unit="deg" if j.joint_type=="hinge" else "m"
            ttk.Label(row,text=f"J{i+1} {j.name or j.joint_type} / {unit}",width=18).pack(side="left")
            var=tk.StringVar(value=f"{user_joints(model,q)[i]:.6g}")
            ttk.Spinbox(row,textvariable=var,from_=j.lower,to=j.upper,
                        increment=1 if j.joint_type=="hinge" else .001,width=10).pack(side="left")
            ttk.Label(row,text=f" [{j.lower:g}, {j.upper:g}]").pack(side="left")
            var.trace_add("write",self._invalidate_pending)
            self.joint_vars.append(var)
            var.trace_add("write",self._update_joint_motor_targets)
        self._update_joint_motor_targets()
        self.channel_input.configure(state="normal")
        self.channel_input.delete(0,"end")
        self.channel_input.insert(0," ".join(map(str,uart["channel_motor_ids"])))
        self.channel_input.configure(state="readonly")
        self.serial_config.set(f"{uart['baudrate']} baud / 8N1 / 无流控\n通道 motor_id 顺序: {uart['channel_motor_ids']}\n当前角度输入顺序: {list(model.motor_ids)} (deg)")
        self.motor_input.delete(0,"end"); self.motor_input.insert(0," ".join(f"{x:g}" for x in model.joints_to_motor(q)))
        self._pose_entries()
        self.redraw()
        self.status.set(f"已应用 {model.dof} 关节模型。黄色箭头为关节正方向；红/绿/蓝为坐标 X/Y/Z。")

    def _update_pose_mode(self,*args):
        only_position=self.position_only.get()
        for entry in self.rpy_entries: entry.configure(state="disabled" if only_position else "normal")
        self.constraint_hint.set("当前仅约束 XYZ；工具方向和自转角可变化，姿态输入不参与求解。" if only_position else
                                 "当前同时约束 XYZ 和 Rx/Ry/Rz；只改 XYZ 仍保留这里的目标姿态。")
        self._invalidate_pending()

    def _update_joint_motor_targets(self,*args):
        if self.model is None or len(self.joint_vars)!=self.model.dof:
            return
        try:
            joints=[float(v.get()) for v in self.joint_vars]
            motors=self.model.joints_to_motor(self.model.user_to_joints(joints))
            self.joint_motor_text.set("\n".join(
                f"电机 ID {i}: {value:+.6f} deg" for i,value in zip(self.model.motor_ids,motors)))
        except (ValueError,TypeError) as exc:
            self.joint_motor_text.set("无法换算："+str(exc))

    def _pose_driver(self, settings):
        preference=self.configured_preference if self.prefer_symmetric.get() else None
        return ArmDriver(self.model,ik_settings=settings,motor_pair_preference=preference)

    def _pose_goal(self):
        """预览与实机计算共用输入规则；仅位置模式完全忽略姿态字段。"""
        position=[float(v.get()) for v in self.xyz]
        if not np.all(np.isfinite(position)): raise ValueError("目标位置必须是有限数值")
        if self.position_only.get():
            return position,None,IKSettings(task_mode="position")
        angles=[float(v.get()) for v in self.rpy]
        if not np.all(np.isfinite(angles)): raise ValueError("目标姿态必须是有限数值")
        return position,Rotation.from_euler("xyz",angles,degrees=True).as_quat(),IKSettings(task_mode="full")

    def use_current_pose(self):
        if not self._idle(): return
        self._pose_entries()
        self.status.set("目标位置和姿态已从当前虚拟模型填入；不是实机反馈，尚未发送。")

    def _pose_entries(self):
        pose=self.model.kinematics.forward(self.q)
        angles=Rotation.from_quat(pose.quaternion_xyzw).as_euler("xyz",degrees=True)
        for var,value in zip(self.xyz,pose.position): var.set(f"{value:.6g}")
        for var,value in zip(self.rpy,angles): var.set(f"{value:.6g}")

    def _submit(self,work):
        if not self._idle(): return
        self._invalidate_pending()
        self.animation=None
        self.discard_result=False
        self.future_kind="preview"
        self.future=self.pool.submit(work)
        self.status.set("正在计算逆解/检查理想路径，界面可以继续旋转视角。")

    def _prepare(self,model,arm,start,goal_user=None,pose=None,check=True):
        current=model.joints_to_motor(start)
        if pose is None:
            target=arm.solve_target_joints(goal_user,current_motor_deg=current)
        else:
            target=arm.solve_target_pose(*pose,current_motor_deg=current)
        goal=model.motor_to_joints(target.target_deg)
        if check: model.collision.require_edge(start,goal)
        return goal

    def solve_pose(self):
        if not self._idle(): return
        try:
            position,quat,settings=self._pose_goal()
            arm=self._pose_driver(settings)
            model,start,check=self.model,self.q.copy(),self.path_check.get()
            self._animation_duration()
            self._submit(lambda:self._prepare(model,arm,start,pose=(position,quat),check=check))
        except Exception as exc: self._error(exc)

    def preview_joints(self):
        if not self._idle(): return
        try:
            goal=[float(v.get()) for v in self.joint_vars]
            model,arm,start,check=self.model,self.arm,self.q.copy(),self.path_check.get()
            self._animation_duration()
            self._submit(lambda:self._prepare(model,arm,start,goal_user=goal,check=check))
        except Exception as exc: self._error(exc)

    def _animation_duration(self):
        value=float(self.duration.get())
        if not np.isfinite(value) or value<=0: raise ValueError("动画时间必须大于 0")
        return value

    def pause(self):
        self.animation=None
        self.discard_result=True
        self.status.set("虚拟动画已暂停；此操作不发送实机停止指令。")

    def reset(self):
        if not self._idle(): return
        self.animation=None; self.target=None; self.q=initial_joints(self.model)
        for var,value in zip(self.joint_vars,user_joints(self.model,self.q)): var.set(f"{value:.6g}")
        self._pose_entries(); self.redraw()
        self.status.set("已回到模型参考姿态，未改变任何实机状态。")

    def apply_motor(self):
        if not self._idle(): return
        try:
            values=[float(v) for v in self.motor_input.get().replace(","," ").split()]
            q=self.model.motor_to_joints(values)
            self.animation=None; self.target=None; self.q=q
            for var,value in zip(self.joint_vars,user_joints(self.model,q)): var.set(f"{value:.6g}")
            self._pose_entries(); self.redraw()
            self.status.set("已按输入角度恢复虚拟姿态；输入值不是串口反馈。")
        except Exception as exc: self._error(exc)

    def copy_frame(self):
        try:
            ids=tuple(int(v) for v in self.channel_input.get().replace(","," ").split())
            sender=UartRxSyncSender(None,channel_motor_ids=ids)
            motors=sender.quantize_angles(self.model.motor_ids,self.model.joints_to_motor(self.q))
            self.model.collision.require_valid(self.model.motor_to_joints(motors))
            frame=sender.encode_angles(self.model.motor_ids,motors)
            self.root.clipboard_clear(); self.root.clipboard_append(frame.hex(" "))
            self.status.set(f"已复制 {len(frame)} 字节帧，未发送。通道顺序：{ids}")
        except Exception as exc: self._error(exc)

    def _on_draw(self,event):
        renderer=getattr(self.ax,"_arm_renderer",None)
        if renderer is not None and renderer.animated: renderer.capture()

    def redraw(self,collisions=None,force_readout=True):
        self.limits=draw_model(self.ax,self.model,self.q,self.target,self.limits)
        renderer=self.ax._arm_renderer
        if not renderer.animated: renderer.enable_blit()
        renderer.blit()
        now=monotonic()
        if not force_readout and now-self.last_readout<.1: return
        self.last_readout=now
        pose=renderer.pose
        if collisions is None: collisions=self.model.collision.contacts(self.q)
        values=user_joints(self.model,self.q)
        angles=Rotation.from_quat(pose.quaternion_xyzw).as_euler("xyz",degrees=True)
        text=(f"Virtual joints [deg/m]: {np.round(values,5).tolist()}\n"
              f"Motor IDs: {self.model.motor_ids}   Motor deg: {np.round(self.model.joints_to_motor(self.q),5).tolist()}\n"
              f"Tool XYZ [m]: {np.round(pose.position,6).tolist()}\n"
              f"Tool computed Rx/Ry/Rz [deg]: {np.round(angles,5).tolist()}\n"
              f"Tool quaternion [xyzw]: {np.round(pose.quaternion_xyzw,6).tolist()}\n"
              f"Geometry: {'COLLISION / insufficient clearance' if collisions else 'current pose clear'} | No physical feedback")
        self.readout.configure(state="normal"); self.readout.delete("1.0","end"); self.readout.insert("end",text)
        self.readout.configure(state="disabled")

    def fit_view(self):
        self.limits=None; self.redraw()

    def tick(self):
        if self.closed: return
        frame_started=monotonic()
        self._poll_serial()
        if self.future is not None and self.future.done():
            future,self.future=self.future,None
            try:
                result=future.result()
                if self.future_kind == "physical":
                    target,snapshot,report=result
                    if snapshot == self._input_snapshot():
                        self.pending_target=target
                        self.pending_snapshot=snapshot
                        self.send_button.configure(state="normal" if self.sender is not None else "disabled")
                        self.send_report.set(report)
                        self.status.set("实机目标已计算。核对电机目标与增量后点击下发；尚未发送。")
                    else:
                        self._invalidate_pending()
                        self.status.set("计算期间输入已改变，请重新计算实机目标。")
                    self.timer=self.root.after(16,self.tick)
                    return
                goal=result
                if self.discard_result:
                    self.timer=self.root.after(16,self.tick)
                    return
                self.target=self.model.kinematics.forward(goal)
                self.animation=(self.q.copy(),goal,monotonic(),self._animation_duration())
                for var,value in zip(self.joint_vars,user_joints(self.model,goal)): var.set(f"{value:.6g}")
                self.limits=None
                self.status.set("正在播放虚拟关节直线插值；这不是固件 S 曲线或实机运动。")
            except Exception as exc: self._error(exc)
        if self.animation is not None:
            start,goal,when,duration=self.animation
            fraction=min(1.,(monotonic()-when)/duration)
            candidate=start+fraction*(goal-start)
            try:
                self.model.joints_to_motor(candidate)
                collisions=self.model.collision.contacts(candidate)
                if collisions: raise ValueError("虚拟姿态发生碰撞或间距不足")
                self.q=candidate; self.redraw(collisions,force_readout=fraction==1)
                if fraction==1:
                    self.animation=None
                    self.status.set("虚拟预览到达目标；实际电机状态未知。可修改目标后再次预览。")
            except Exception as exc:
                self.animation=None; self._error(exc)
        elapsed=monotonic()-frame_started
        self.timer=self.root.after(max(1,int(1000*(1/60-elapsed))),self.tick)

    def open_config(self):
        if not self._idle(): return
        if self.sender is not None:
            self.status.set("打开其他模型前请先断开串口。")
            return
        name=filedialog.askopenfilename(filetypes=[("JSON","*.json")])
        if not name: return
        try:
            source=Path(name).read_text(encoding="utf-8")
            load_configuration(json.loads(source))
            self.editor.delete("1.0","end"); self.editor.insert("1.0",source)
            self.path=Path(name); self.apply_model()
        except Exception as exc: self._error(exc)

    def save_config(self):
        try:
            data=json.loads(self.editor.get("1.0","end")); load_configuration(data)
            name=filedialog.asksaveasfilename(defaultextension=".json",initialdir=self.path.parent,
                                             initialfile=self.path.name,filetypes=[("JSON","*.json")])
            if name:
                self.path=Path(name)
                self.path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
                self.status.set(f"已保存 {self.path}；编辑后还需点击校验并应用参数。")
        except Exception as exc: self._error(exc)

    def export_model(self):
        try:
            data={"model":model_description(self.model),"uart":self.uart_settings}
            if "control" in self.applied_description:
                data["control"]=self.applied_description["control"]
            name=filedialog.asksaveasfilename(defaultextension=".json",initialfile="arm_config.json")
            if name: Path(name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        except Exception as exc: self._error(exc)

    def open_guide(self):
        window=tk.Toplevel(self.root); window.title("ArmDriver 使用说明"); window.geometry("900x700")
        text=tk.Text(window,wrap="word",font=("Microsoft YaHei UI",10)); text.pack(fill="both",expand=True)
        text.insert("1.0",(ROOT/"README.md").read_text(encoding="utf-8")); text.configure(state="disabled")

    def close(self):
        if self.closed: return
        self.closed=True
        self.root.after_cancel(self.timer)
        idle_draw=getattr(self.canvas,"_idle_draw_id",None)
        if idle_draw is not None:
            self.canvas.get_tk_widget().after_cancel(idle_draw)
            self.canvas._idle_draw_id=None
        self.cancel_send.set()
        # 清理排在串口任务之后，避免在另一线程写半帧时关闭连接。
        sender,future,action=self.sender,self.serial_future,self.serial_action
        def cleanup():
            connection=sender
            if action=="connect" and future is not None:
                try: connection=future.result()
                except Exception: pass
            if connection is not None: connection.close()
        self.serial_pool.submit(cleanup)
        self.serial_pool.shutdown(wait=False)
        self.pool.shutdown(wait=False,cancel_futures=True)
        self.root.destroy()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path)
    args=parser.parse_args(argv)
    data=json.loads(args.config.read_text(encoding="utf-8-sig")) if args.config else None
    root=tk.Tk()
    style=ttk.Style(root)
    if "vista" in style.theme_names(): style.theme_use("vista")
    app=DebugApp(root,data)
    if args.config: app.path=args.config.resolve()
    try:
        root.mainloop()
    finally:
        app.close()


if __name__=="__main__":
    main()
