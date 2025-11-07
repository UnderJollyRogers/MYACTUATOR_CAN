#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TeachPendant_tk.py — Minimal Tkinter Teach Pendant GUI for a Delta Robot

Features
- Cartesian jog: X±, Y±, Z± (mm)
- Joint jog: J1±, J2±, J3± (deg)
- Mode selector (Cartesian / Joint) + step sizes
- Gripper Open/Close toggle
- Home (Go to Home)
- Emergency Stop (E‑STOP)
- Status readouts + message log
- Keyboard shortcuts
    * Cartesian mode: ←/→ = X−/X+, ↑/↓ = Y+/Y− (robot coords), PgUp/PgDn = Z+/Z−
    * Joint mode: 1/2/3 = J1+/J2+/J3+ and Shift+1/2/3 = J1−/J2−/J3−

Run
    python3 TeachPendant_tk.py

Notes
- RobotInterface below is a stub. Replace with your real backend (CAN/Serial/ROS).
- All jogs are step-based (one step per click/keypress). Implement hold-to-jog with key repeat if desired.
"""

import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass

# ------------------------------
# Robot interface (stub)
# ------------------------------
@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    j: tuple = (0.0, 0.0, 0.0)  # J1, J2, J3 (deg)
    gripper_open: bool = True

class RobotInterface:
    def __init__(self):
        self.state = RobotState()
        self.connected = True

    # --- Cartesian ---
    def move_cartesian(self, dx=0, dy=0, dz=0) -> bool:
        s = self.state
        s.x += dx; s.y += dy; s.z += dz
        return True

    # --- Joints ---
    def move_joint(self, idx: int, ddeg: float) -> bool: 
        # I do not understant
        s = list(self.state.j)
        print(s)
        s[idx] += ddeg
        self.state.j = tuple(s)
        return True

    # --- Gripper ---
    def set_gripper(self, open_: bool) -> bool:
        self.state.gripper_open = open_
        return True

    # --- Home / E‑stop ---
    def go_home(self) -> bool:
        self.state = RobotState(gripper_open=self.state.gripper_open)
        return True

    def estop(self) -> bool:
        # Replace with motor disable + controller halt
        return True

# ------------------------------
# GUI
# ------------------------------
class PendantGUI(tk.Tk):
    def __init__(self, robot: RobotInterface):
        super().__init__()
        self.title("Delta Teach Pendant — Tkinter")
        self.geometry("960x600")
        self.minsize(880, 520)
        self.robot = robot
        self.mode = tk.StringVar(value="Cartesian")
        self.step_cart = tk.DoubleVar(value=1.0)
        self.step_joint = tk.DoubleVar(value=1.0)
        self.gripper_open = tk.BooleanVar(value=True)

        self._build_style()
        self._build_layout()
        self._bind_keys()
        self._refresh_status()

    # ---------- Style ----------
    def _build_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("Danger.TButton", foreground="white", background="#d32f2f")
        style.map("Danger.TButton", background=[("active", "#b71c1c")])

    # ---------- Layout ----------
    def _build_layout(self):
        container = ttk.Frame(self, padding=8)
        container.pack(fill=tk.BOTH, expand=True)

        # Top bar
        top = ttk.Frame(container)
        top.pack(fill=tk.X, pady=(0,6))

        status_lbl = ttk.Label(top, text="Connected" if self.robot.connected else "Disconnected")
        status_lbl.configure(foreground="#2e7d32" if self.robot.connected else "#b71c1c")
        ttk.Label(top, text="Status:").pack(side=tk.LEFT)
        status_lbl.pack(side=tk.LEFT, padx=(4,16))

        ttk.Label(top, text="Mode:").pack(side=tk.LEFT)
        mode_cb = ttk.Combobox(top, textvariable=self.mode, values=["Cartesian", "Joint"], width=10, state="readonly")
        mode_cb.bind("<<ComboboxSelected>>", lambda e: self._status("Mode: " + self.mode.get()))
        mode_cb.pack(side=tk.LEFT, padx=(4,16))

        ttk.Label(top, text="Step (mm):").pack(side=tk.LEFT)
        stepc = ttk.Spinbox(top, textvariable=self.step_cart, from_=0.01, to=100.0, increment=0.01, width=8)
        stepc.pack(side=tk.LEFT, padx=(4,16))

        ttk.Label(top, text="Step (deg):").pack(side=tk.LEFT)
        stepj = ttk.Spinbox(top, textvariable=self.step_joint, from_=0.1, to=30.0, increment=0.1, width=8)
        stepj.pack(side=tk.LEFT, padx=(4,16))

        home_btn = ttk.Button(top, text="🏠 Home", command=self.on_home)
        home_btn.pack(side=tk.LEFT, padx=(4,8))

        self.grip_btn = ttk.Checkbutton(top, text="Gripper: OPEN", variable=self.gripper_open, command=self.on_gripper, style="TCheckbutton")
        self.grip_btn.pack(side=tk.LEFT, padx=(4,8))

        estop_btn = ttk.Button(top, text="🛑 E‑STOP", style="Danger.TButton", command=self.on_estop)
        estop_btn.pack(side=tk.LEFT, padx=(8,0))

        # Middle panels
        middle = ttk.Frame(container)
        middle.pack(fill=tk.BOTH, expand=True)
        middle.columnconfigure(0, weight=1)
        middle.columnconfigure(1, weight=1)

        cart = self._cartesian_group(middle)
        cart.grid(row=0, column=0, sticky="nsew", padx=(0,6), pady=6)

        joint = self._joint_group(middle)
        joint.grid(row=0, column=1, sticky="nsew", padx=(6,0), pady=6)

        # Status and Log
        bottom = ttk.Frame(container)
        bottom.pack(fill=tk.BOTH, expand=True)
        bottom.columnconfigure(1, weight=1)
        ttk.Label(bottom, text="Pose:").grid(row=0, column=0, sticky="w")
        self.lbl_pos = ttk.Label(bottom, text="P = (0.00, 0.00, 0.00) mm")
        self.lbl_pos.grid(row=0, column=1, sticky="w")

        ttk.Label(bottom, text="Joints:").grid(row=1, column=0, sticky="w")
        self.lbl_jnt = ttk.Label(bottom, text="J = (0.00, 0.00, 0.00) deg")
        self.lbl_jnt.grid(row=1, column=1, sticky="w")

        ttk.Label(bottom, text="Log:").grid(row=2, column=0, sticky="nw", pady=(6,0))
        self.txt_log = tk.Text(bottom, height=10, wrap="word", state="disabled")
        self.txt_log.grid(row=2, column=1, sticky="nsew", pady=(6,0))
        bottom.rowconfigure(2, weight=1)

    def _cartesian_group(self, parent):
        box = ttk.LabelFrame(parent, text="Cartesian Jog (mm)")
        grid = ttk.Frame(box)
        grid.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        def b(txt, r, c, cmd):
            btn = ttk.Button(grid, text=txt, command=cmd)
            btn.grid(row=r, column=c, sticky="nsew", padx=4, pady=4)
            return btn

        b("Y+", 0, 1, lambda: self.jog_cart(0, +1, 0))
        b("X-", 1, 0, lambda: self.jog_cart(-1, 0, 0))
        b("Z+", 1, 1, lambda: self.jog_cart(0, 0, +1))
        b("X+", 1, 2, lambda: self.jog_cart(+1, 0, 0))
        b("Y-", 2, 1, lambda: self.jog_cart(0, -1, 0))
        b("Z-", 3, 1, lambda: self.jog_cart(0, 0, -1))

        for r in range(4):
            grid.rowconfigure(r, weight=1)
        for c in range(3):
            grid.columnconfigure(c, weight=1)
        return box

    def _joint_group(self, parent):
        box = ttk.LabelFrame(parent, text="Joint Jog (deg)")
        grid = ttk.Frame(box)
        grid.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        for i in range(3):
            ttk.Label(grid, text=f"J{i+1}").grid(row=i, column=0, sticky="e", padx=(0,6))
            btn_minus = ttk.Button(grid, text="-", width=6, command=lambda k=i: self.jog_joint(k, -1))
            btn_plus  = ttk.Button(grid, text="+", width=6, command=lambda k=i: self.jog_joint(k, +1))
            btn_minus.grid(row=i, column=1, sticky="ew", padx=4, pady=4)
            btn_plus .grid(row=i, column=2, sticky="ew", padx=4, pady=4)

        for c in range(3):
            grid.columnconfigure(c, weight=1)
        return box

    # ---------- Key Bindings ----------
    def _bind_keys(self):
        # General
        self.bind_all("<Escape>", lambda e: self.on_estop())

        # Cartesian
        self.bind_all("<Left>",  lambda e: self._accel_cart(dx=-1))
        self.bind_all("<Right>", lambda e: self._accel_cart(dx=+1))
        self.bind_all("<Up>",    lambda e: self._accel_cart(dy=+1))
        self.bind_all("<Down>",  lambda e: self._accel_cart(dy=-1))
        self.bind_all("<Prior>", lambda e: self._accel_cart(dz=+1))   # PageUp
        self.bind_all("<Next>",  lambda e: self._accel_cart(dz=-1))   # PageDown

        # Joint (numbers for plus, Shift+numbers for minus)
        self.bind_all("1", lambda e: self._accel_joint(0, +1))
        self.bind_all("2", lambda e: self._accel_joint(1, +1))
        self.bind_all("3", lambda e: self._accel_joint(2, +1))
        self.bind_all("!", lambda e: self._accel_joint(0, -1))
        self.bind_all("@", lambda e: self._accel_joint(1, -1))
        self.bind_all("#", lambda e: self._accel_joint(2, -1))

    # ---------- Helpers ----------
    def _status(self, msg):
        self._log(msg)

    def _log(self, msg):
        self.txt_log.configure(state="normal")
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)
        self.txt_log.configure(state="disabled")

    def _refresh_status(self):
        s = self.robot.state
        self.lbl_pos.configure(text=f"P = ({s.x:.2f}, {s.y:.2f}, {s.z:.2f}) mm")
        self.lbl_jnt.configure(text=f"J = ({s.j[0]:.2f}, {s.j[1]:.2f}, {s.j[2]:.2f}) deg")

    # ---------- Actions ----------
    def jog_cart(self, sx=0, sy=0, sz=0):
        if self.mode.get() != "Cartesian":
            self._status("Switch to Cartesian mode to use these.")
            return
        step = float(self.step_cart.get())
        self.robot.move_cartesian(step*sx, step*sy, step*sz)
        self._log(f"Cartesian jog: dX={step*sx:.3f} mm, dY={step*sy:.3f} mm, dZ={step*sz:.3f} mm")
        self._refresh_status()

    def jog_joint(self, joint_idx: int, sign: int):
        if self.mode.get() != "Joint":
            self._status("Switch to Joint mode to use these.")
            return
        step = float(self.step_joint.get())
        self.robot.move_joint(joint_idx, sign*step)
        self._log(f"Joint J{joint_idx+1} jog: dθ={sign*step:.3f} deg")
        self._refresh_status()

    def on_gripper(self):
        #Funciòn que abre y cierra el gripper
        open_ = bool(self.gripper_open.get())
        self.robot.set_gripper(open_)
        self.grip_btn.configure(text="Gripper: OPEN" if open_ else "Gripper: CLOSE")
        self._log("Gripper: OPEN" if open_ else "Gripper: CLOSE")
        

    def on_home(self):
        self.robot.go_home()
        self._log("Go Home: pose reset (0,0,0) and joints (0,0,0)")
        self._refresh_status()

    def on_estop(self):
        self.robot.estop()
        self._log("E‑STOP ACTIVATED! Motors disabled. (stub)")

    # Accelerator helpers respecting mode
    def _accel_cart(self, dx=0, dy=0, dz=0):
        if self.mode.get() == "Cartesian":
            self.jog_cart(dx, dy, dz)
        else:
            self._status("Accelerator ignored (not in Cartesian mode)")

    def _accel_joint(self, idx, sign):
        if self.mode.get() == "Joint":
            self.jog_joint(idx, sign)
        else:
            self._status("Accelerator ignored (not in Joint mode)")

# ------------------------------
# Entry
# ------------------------------
if __name__ == "__main__":
    robot = RobotInterface()
    app = PendantGUI(robot)
    app.mainloop()
