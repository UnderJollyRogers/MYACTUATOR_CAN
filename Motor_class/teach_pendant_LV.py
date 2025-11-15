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
from tkinter import ttk, messagebox 
from dataclasses import dataclass
from inverse_kinematics_opt import InverseKinematic_Opt
from direct_kinematic import DirectKinematic 
from can_motor import can_motor
import numpy as np

import asyncio
import threading
import time
from bleak import BleakClient, BleakScanner

# UUIDs (deben coincidir con el ESP32)
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# ------------------------------
# BLE Interface (CORREGIDA PARA LINUX)
# ------------------------------
class BLEInterface:
    """Interfaz BLE para ESP32-S3 - Compatible con Linux"""
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.device_name = "ESP32_Gripper"
        self.device_address = None
        self.response = None
        self.loop = None
        self.loop_thread = None
        self._start_loop()
    
    def _start_loop(self):
        """Inicia un event loop dedicado en un thread separado"""
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()
        
        # Esperar a que el loop esté listo
        while self.loop is None:
            time.sleep(0.01)
    
    def _run_coro(self, coro):
        """Ejecuta una coroutine en el event loop dedicado"""
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        try:
            return future.result(timeout=15.0)
        except Exception as e:
            print(f"Error ejecutando coroutine: {e}")
            return None
        
    async def scan_devices(self):
        """Escanea dispositivos BLE"""
        print("Escaneando dispositivos BLE...")
        devices = await BleakScanner.discover(timeout=5.0)
        
        found_devices = []
        for device in devices:
            print(f"Encontrado: {device.name} - {device.address}")
            if device.name and self.device_name in device.name:
                found_devices.append({
                    'name': device.name,
                    'address': device.address
                })
        
        return found_devices
    
    async def connect_async(self, address=None):
        """Conecta al dispositivo BLE"""
        try:
            if address is None:
                devices = await self.scan_devices()
                if not devices:
                    return False, "No se encontró ESP32_Gripper"
                address = devices[0]['address']
            
            self.device_address = address
            print(f"Conectando a {address}...")
            
            self.client = BleakClient(address, timeout=10.0)
            await self.client.connect()
            
            if self.client.is_connected:
                self.connected = True
                print("✓ Conectado exitosamente")
                
                # Suscribirse a notificaciones
                await self.client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                
                return True, f"Conectado a {address}"
            else:
                return False, "No se pudo conectar"
                
        except Exception as e:
            print(f"Error de conexión: {e}")
            return False, f"Error: {str(e)}"
    
    def notification_handler(self, sender, data):
        """Maneja las notificaciones BLE del ESP32"""
        try:
            self.response = data.decode('utf-8')
            print(f"Notificación recibida: {self.response}")
        except Exception as e:
            print(f"Error procesando notificación: {e}")
    
    async def disconnect_async(self):
        """Desconecta del dispositivo"""
        try:
            if self.client and self.client.is_connected:
                await self.client.stop_notify(CHARACTERISTIC_UUID)
                await self.client.disconnect()
        except Exception as e:
            print(f"Error desconectando: {e}")
        finally:
            self.connected = False
    
    async def send_command_async(self, command):
        """Envía comando al ESP32 y espera respuesta"""
        if not self.connected or not self.client:
            print("No conectado")
            return None
        
        try:
            # Limpiar respuesta anterior
            self.response = None
            
            # Enviar comando
            print(f"Enviando comando: {command}")
            await self.client.write_gatt_char(
                CHARACTERISTIC_UUID, 
                command.encode(),
                response=True  # Cambiado a True para mejor compatibilidad en Linux
            )
            
            # Esperar respuesta (con timeout)
            timeout = 3.0
            elapsed = 0
            while self.response is None and elapsed < timeout:
                await asyncio.sleep(0.1)
                elapsed += 0.1
            
            if self.response:
                print(f"Respuesta: {self.response}")
                return self.response
            else:
                print("Timeout esperando respuesta")
                # Intenta leer directamente si no llegó notificación
                try:
                    response_data = await self.client.read_gatt_char(CHARACTERISTIC_UUID)
                    response = response_data.decode('utf-8')
                    print(f"Respuesta leída: {response}")
                    return response
                except Exception as e:
                    print(f"Error leyendo respuesta: {e}")
                    return None
            
        except Exception as e:
            print(f"Error enviando comando: {e}")
            return None
    
    def connect(self, address=None):
        """Wrapper sincrónico para conectar"""
        return self._run_coro(self.connect_async(address))
    
    def disconnect(self):
        """Wrapper sincrónico para desconectar"""
        if self.connected:
            self._run_coro(self.disconnect_async())
    
    def send_command(self, command):
        """Wrapper sincrónico para enviar comando"""
        return self._run_coro(self.send_command_async(command))
    
    def scan(self):
        """Wrapper sincrónico para escanear"""
        return self._run_coro(self.scan_devices())



# ---------- Hold-to-repeat utility ----------



class RepeatButton(ttk.Button):
    """A ttk.Button that repeats its command while the mouse is held down."""
    def __init__(self, master=None, *, command=None, first_interval=100, interval=1, **kw):
        super().__init__(master, **kw)
        self._repeat_cmd = command or (lambda: None)
        self._first_interval = first_interval
        self._interval = interval
        self._job = None
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Leave>", self._on_release)

    def _on_press(self, _evt=None):
        # Run once immediately, then schedule repeats
        self._repeat_cmd()
        self._job = self.after(self._first_interval, self._repeat)

    def _repeat(self):
        self._repeat_cmd()
        self._job = self.after(self._interval, self._repeat)

    def _on_release(self, _evt=None):
        if self._job is not None:
            self.after_cancel(self._job)
            self._job = None

# ------------------------------
# Robot interface (stub)
# ------------------------------
@dataclass
class RobotState:
    j: tuple = (0.0, 0.0, 0.0)  # J1, J2, J3 (deg)
    lst = list(j)
    X = DirectKinematic(lst) 
    x: float = X[0]
    y: float = X[1]
    z: float = X[2]
    gripper_open: bool = True
    min_angle: float = 0

class RobotInterface:
    def __init__(self):
        self.state = RobotState()
        self.connected = can_motor.is_connect
        self.motor1 = can_motor(motor_id=1, max_speed=500)
        self.motor2 = can_motor(motor_id=2, max_speed=500)
        self.motor3 = can_motor(motor_id=3, max_speed=500)
        self.ble = BLEInterface()
        self.ble_connected = False


   #BLE 
    
    def connect_ble(self, address=None):
        """Conecta al ESP32 vía BLE"""
        success, message = self.ble.connect(address)
        self.connected = success
        return success, message
        
    def disconnect_ble(self):
        """Desconecta del ESP32"""
        self.ble.disconnect()
        self.connected = False
            
    # --- Gripper ---
    def set_gripper(self, open_: bool) -> bool:
        """Controla el gripper vía BLE"""
        if not self.connected:
            print("No conectado al ESP32")
            return False
        
        command = 'O' if open_ else 'C'
        print(f"Enviando comando de gripper: {command}")
        
        response = self.ble.send_command(command)
        
        print(f"Respuesta recibida: {response}")
        
        # Verificar si la respuesta es válida
        if response:
            if 'OK:OPEN' in response or 'OK:CLOSE' in response:
                self.state.gripper_open = open_
                print(f"✓ Gripper {'ABIERTO' if open_ else 'CERRADO'} exitosamente")
                return True
        
        print("✗ Respuesta inválida o timeout")
        return False

    # --- Cartesian ---
    def move_cartesian(self, dx=0, dy=0, dz=0) -> bool:
        x = self.state.x
        y = self.state.y
        z = self.state.z
        j = self.state.j
        print([x, y, z])
        x += dx; y += dy; z += dz
        J = InverseKinematic_Opt([x, y, z], list(j))
        self.state.j = tuple(J)
        if (self.connected):
            self.motor1.absolute_position_control(J[0])
            self.motor2.absolute_position_control(J[1])
            self.motor3.absolute_position_control(J[2])
        self.state.x = x
        self.state.y = y
        self.state.z = z
        return True

    # --- Joints ---
    def move_joint(self, idx: int, ddeg: float) -> bool: 
        s = list(self.state.j)
        s[idx] += ddeg
        self.state.j = tuple(s)
        lst = list(s)
        X = DirectKinematic(lst, [self.state.x,self.state.y,self.state.z]) 
        self.state.x = X[0]
        self.state.y = X[1]
        self.state.z = X[2]
        if (self.connected):
                self.motor1.absolute_position_control(s[0])
                self.motor2.absolute_position_control(s[1])
                self.motor3.absolute_position_control(s[2])
        return True


    # --- Home / E‑stop ---
    def go_home(self) -> bool:
        self.state = RobotState(gripper_open=self.state.gripper_open)
        if (self.connected):
                self.motor1.absolute_position_control(0)
                self.motor2.absolute_position_control(0)
                self.motor3.absolute_position_control(0)
        return True

    def estop(self) -> bool:
        # Replace with motor disable + controller halt
        if (self.connected):
                self.motor1.stop_motor
                self.motor2.stop_motor
                self.motor3.stop_motor
        return True
    
    def write_current_position_as_zero(self) -> bool:
    # Persist the current encoder position as zero on the controller (stub).
    # TODO: call your motor API, e.g., my_motor.write_current_position_as_zero()
    # Here we reset joint state to 0 but keep the cartesian estimate as-is.
        if self.connected:
            self.motor1.write_current_position_as_zero()
            self.motor2.write_current_position_as_zero()
            self.motor3.write_current_position_as_zero()
        X = DirectKinematic([0,0,0]) 
        x: float = X[0]
        y: float = X[1]
        z: float = X[2]
        self.state = RobotState(
            x=x, y=y, z=z,
            j=(0.0, 0.0, 0.0),
            gripper_open=self.state.gripper_open
        )
        return True

# ------------------------------
# GUI
# ------------------------------
class PendantGUI(tk.Tk):
    def __init__(self, robot: RobotInterface):
        super().__init__()
        self.title("Delta Teach Pendant — BLE Gripper (ESP32-S3)")
        self.geometry("960x600")
        self.minsize(880, 600)
        self.robot = robot
        self.mode = tk.StringVar(value="Cartesian")
        self.step_cart = tk.DoubleVar(value=1.0)
        self.step_joint = tk.DoubleVar(value=1.0)
        self.gripper_open = tk.BooleanVar(value=True)
        self.selected_device = tk.StringVar()

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
        
        
        # BLE Connection Bar
        ble_frame = ttk.LabelFrame(container, text="BLE Connection (ESP32-S3)", padding=8)
        ble_frame.pack(fill=tk.X, pady=(0,6))
        
        status_frame = ttk.Frame(ble_frame)
        status_frame.pack(fill=tk.X, pady=(0,8))
        
        self.ble_status_lbl = ttk.Label(status_frame, text="Desconectado", 
                                        foreground="#b71c1c", font=("", 10, "bold"))
        self.ble_status_lbl.pack(side=tk.LEFT, padx=(0,10))
        
        device_frame = ttk.Frame(ble_frame)
        device_frame.pack(fill=tk.X, pady=(0,8))
        
        ttk.Label(device_frame, text="Dispositivo:").pack(side=tk.LEFT, padx=(0,5))
        self.device_combo = ttk.Combobox(device_frame, textvariable=self.selected_device, 
                                         width=35, state="readonly")
        self.device_combo.pack(side=tk.LEFT, padx=(0,10))
        
        self.scan_btn = ttk.Button(device_frame, text="Escanear BLE", 
                                   command=self.on_scan_ble, width=15)
        self.scan_btn.pack(side=tk.LEFT, padx=(0,5))
        
        btn_frame = ttk.Frame(ble_frame)
        btn_frame.pack(fill=tk.X)
        
        self.connect_btn = ttk.Button(btn_frame, text="Conectar", 
                                      command=self.on_connect_ble,
                                      style="Success.TButton", width=15)
        self.connect_btn.pack(side=tk.LEFT, padx=(0,5))
        
        self.disconnect_btn = ttk.Button(btn_frame, text="Desconectar", 
                                         command=self.on_disconnect_ble,
                                         state="disabled", width=15)
        self.disconnect_btn.pack(side=tk.LEFT)

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


        top2 = ttk.Frame(container)
        top2.pack(fill=tk.X, pady=(0,6))

        self.grip_btn = ttk.Checkbutton(
            top2,
            text="Gripper: OPEN",
            variable=self.gripper_open,
            command=self.on_gripper
        )
        self.grip_btn.pack(side=tk.LEFT, padx=(4,8))

        estop_btn = ttk.Button(
            top2, text="E-STOP",
            style="Danger.TButton",
            command=self.on_estop
        )
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
            btn = RepeatButton(grid, text=txt, command=cmd)
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
            btn_minus = RepeatButton(grid, text="-", width=6, command=lambda k=i: self.jog_joint(k, -1))
            btn_plus  = RepeatButton(grid, text="+", width=6, command=lambda k=i: self.jog_joint(k, +1))
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

    
   # ----BLE ACTIONS------
   
    # ---------- BLE Actions ----------
    def on_scan_ble(self):
        """Escanear dispositivos BLE"""
        self._log("Escaneando dispositivos BLE...")
        self.scan_btn.configure(state="disabled", text="Escaneando...")
        
        def scan_thread():
            devices = self.robot.ble.scan()
            self.after(0, lambda: self._update_device_list(devices))
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def _update_device_list(self, devices):
        """Actualiza lista de dispositivos"""
        self.scan_btn.configure(state="normal", text="Escanear BLE")
        
        if not devices:
            self.device_combo['values'] = ["No se encontro ESP32_Gripper"]
            self.device_combo.current(0)
            self._log("No se encontro ESP32_Gripper")
            messagebox.showinfo("Escaneo BLE", 
                              "No se encontro el dispositivo ESP32_Gripper.\n\n"
                              "Verifica que el ESP32-S3 este encendido.")
            return
        
        device_list = [f"{d['name']} ({d['address']})" for d in devices]
        self.device_combo['values'] = device_list
        self.device_combo.current(0)
        self._log(f"Encontrado: {devices[0]['name']}")
    
    def on_connect_ble(self):
        """Conectar al ESP32-S3"""
        selected = self.selected_device.get()
        
        if not selected or "No se encontro" in selected:
            messagebox.showwarning("Sin Dispositivo", 
                                 "Escanea primero para encontrar el ESP32_Gripper")
            return
        
        # Extraer dirección MAC
        address = selected.split("(")[1].split(")")[0]
        
        self._log(f"Conectando a {address}...")
        self.connect_btn.configure(state="disabled", text="Conectando...")
        
        def connect_thread():
            success, message = self.robot.connect_ble(address)
            self.after(0, lambda: self._update_connection_status(success, message))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def _update_connection_status(self, success, message):
        """Actualiza estado de conexión"""
        if success:
            self.ble_status_lbl.configure(text="Conectado BLE", foreground="#2e7d32")
            self.connect_btn.configure(state="disabled", text="Conectar")
            self.disconnect_btn.configure(state="normal")
            self.device_combo.configure(state="disabled")
            self.scan_btn.configure(state="disabled")
            self._log(f"{message}")
            messagebox.showinfo("Conexion Exitosa", 
                              f"{message}\n\nYa puedes controlar el gripper!")
        else:
            self.ble_status_lbl.configure(text="Error", foreground="#b71c1c")
            self.connect_btn.configure(state="normal", text="Conectar")
            self._log(f"{message}")
            messagebox.showerror("Error BLE", message)
    
    def on_disconnect_ble(self):
        """Desconectar del ESP32"""
        self.robot.disconnect_ble()
        self.ble_status_lbl.configure(text="Desconectado", foreground="#b71c1c")
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")
        self.device_combo.configure(state="readonly")
        self.scan_btn.configure(state="normal")
        self._log("Desconectado del ESP32-S3")

    
    
    
    
    
    
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
        check = self.robot.move_cartesian(step*sx, step*sy, step*sz)
        if check:
            self._log(f"Cartesian jog: dX={step*sx:.3f} mm, dY={step*sy:.3f} mm, dZ={step*sz:.3f} mm")
            self._refresh_status()
        else:
            self._log("Intenting to move over max angle.")

    def jog_joint(self, joint_idx: int, sign: int):
        s = self.robot.state
        if self.mode.get() != "Joint":
            self._status("Switch to Joint mode to use these.")
            return
        step = float(self.step_joint.get())
        self.robot.move_joint(joint_idx, sign*step)
        self._log(f"Joint J{joint_idx+1} jog: dθ={sign*step:.3f} deg")
        self._refresh_status()

    def on_gripper(self):
        """Función que abre y cierra el gripper vía BLE"""
        if not self.robot.connected:
            messagebox.showwarning("No Conectado", 
                                 "Conecta al ESP32-S3 primero\npara controlar el gripper")
            self.gripper_open.set(not self.gripper_open.get())
            return
        
        open_ = bool(self.gripper_open.get())
        
        def gripper_thread():
            success = self.robot.set_gripper(open_)
            status_text = "OPEN" if open_ else "CLOSE"
            
            if success:
                self.after(0, lambda: self.grip_btn.configure(
                    text=f"Gripper: {status_text}"))
                self.after(0, lambda: self._log(f"Gripper: {status_text}"))
            else:
                self.after(0, lambda: self._log(f"Error al controlar gripper"))
                self.after(0, lambda: messagebox.showerror(
                    "Error", "No se pudo controlar el gripper.\n"
                    "Verifica la conexion BLE."))
                self.after(0, lambda: self.gripper_open.set(not open_))
        
        threading.Thread(target=gripper_thread, daemon=True).start()


    def on_home(self):
        self.robot.go_home()
        self._log("Go Home: pose reset (0,0,0) and joints (0,0,0)")
        self._refresh_status()

    def on_zero(self):
        ok = self.robot.write_current_position_as_zero()
        if ok:
            self._log("Zeroed: current joint positions written as 0 (encoder zero).")
        else:
            self._log("Failed to write current position as zero.")
        self._refresh_status()

    def on_estop(self):
        self.robot.estop()
        self._log("E‑STOP ACTIVATED! Motors disabled. (stub)")

# ------------------------------
# Entry
# ------------------------------
if __name__ == "__main__":
    print("=" * 50)
    print("Delta Teach Pendant - BLE Gripper (ESP32-S3)")
    print("=" * 50)
    robot = RobotInterface()
    app = PendantGUI(robot)
    app.mainloop()
