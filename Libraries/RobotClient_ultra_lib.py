#Dany_Libraries
#Code partially based on Mariana's work
import socket
import time
import logging
import rtde_control
import rtde_receive
import rtde_io
import json
import os
import cv2
import numpy as np

class RobotClient:

    def __init__(self):
        
        if not os.path.exists("config.json"):
            print("Archivo config.json no encontrado, Corre HandEye_hyper_lib para crear el archivo..")
            exit()
        with open("config.json", "r") as f:
            self.config = json.load(f)
        self.ROBOT_IP = self.config["General_config"]["ROBOT_IP"]
        self.PORT = 29999
        self.program_name = ''
        self.rtde_c = None  # None = desconectado
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        self.rtde_io_iface = rtde_io.RTDEIOInterface(self.ROBOT_IP)

    # ── Estado interno ─────────────────────────────────────────────────────────
    def _connect_control(self):
        """Reconecta rtde_c si está desconectado. Siempre espera 0.6s antes."""
        if self.rtde_c is None:
            time.sleep(0.6)  # FIX: el robot necesita este tiempo después de liberar el socket del dashboard
            self.rtde_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)
            logging.info("RTDEControl conectado.")

    def _disconnect_control(self):
        """Desconecta rtde_c si está conectado. El sleep va DENTRO de send_dashboard_command."""
        if self.rtde_c is not None:
            self.rtde_c.disconnect()
            self.rtde_c = None
            # FIX: NO sleep aquí — el sleep va antes de abrir el socket dashboard
            logging.info("RTDEControl desconectado.")

    # ── Dashboard (desconecta rtde_c automáticamente antes de usar socket) ─────
    def send_dashboard_command(self, command):
        self._disconnect_control()  # desconecta RTDE si está activo
        time.sleep(0.6)             # FIX: esperar siempre antes de abrir socket dashboard
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5.0)
                s.connect((self.ROBOT_IP, self.PORT))
                s.recv(1024)
                s.sendall((command + '\n').encode('utf-8'))
                return s.recv(1024).decode('utf-8').strip()
        except Exception as e:
            logging.error(f"Error de conexión: {e}")
            return ""

    def load_program(self, program_name):
        self.program_name = program_name
        response = self.send_dashboard_command(f"load {program_name}")
        # FIX: el UR responde "Loading program: nombre.urp", NO "Loaded"
        if "Loading program" not in response:
            logging.warning("Reintentando load...")
            time.sleep(1)
            self.send_dashboard_command(f"load {program_name}")
        time.sleep(1)

    def play(self):
        response = self.send_dashboard_command("play")
        if "Starting program" not in response:
            logging.warning("Reintentando play...")
            time.sleep(1)
            self.send_dashboard_command("play")
        time.sleep(1)

    def stop_program(self):
        self.send_dashboard_command("stop")
        self.send_dashboard_command("unlock protective stop")
        self.send_dashboard_command("reset protective stop")
        time.sleep(1)

    def wait_for_program_done(self, bit=7, poll=0.1, timeout=30):
        """
        Espera a que el bit de salida digital indique que el programa UR terminó.

        Retorna:
            True  → el bit subió antes del timeout (programa terminó normalmente)
            False → expiró el timeout (programa colgado / no completó)
        """
        t_start = time.time()
        while True:
            bits = self.rtde_r.getActualDigitalOutputBits()
            if (bits >> bit) & 1:
                logging.info("Programa terminado.")
                self.rtde_io_iface.setStandardDigitalOut(bit, False)  # reset para la próxima vez
                return True
            if (time.time() - t_start) > timeout:
                logging.warning(f"Timeout {timeout:.1f}s esperando DO[{bit}].")
                # NO se resetea el bit aquí: el robot podría seguir trabajando.
                # Se intenta detener el programa para liberar la celda.
                try:
                    self.send_dashboard_command("stop")
                except Exception:
                    pass
                return False
            time.sleep(poll)

    def run_program_and_wait(self, program_name, bit=7) -> bool:
        """
        Carga y corre un programa UR y espera a que termine.

        Retorna:
            True  → el programa terminó normalmente (DO[bit] subió antes de timeout)
            False → el programa NO terminó dentro del timeout (URP colgado o falló)

        Al salir, rtde_c está desconectado — llama moveL después para reconectar.
        """
        self.load_program(program_name)
        self.play()
        finished = self.wait_for_program_done(bit=bit)
        # FIX: el sleep crítico después de que termina el dashboard va aquí,
        # NO en _connect_control, para que esté siempre presente después de run_program_and_wait
        time.sleep(0.6)
        return finished

    # ── RTDE (conecta rtde_c automáticamente) ──────────────────────────────────
    def moveL(self, pose, speed=0.1, accel=0.1):
        self._connect_control()
        self.rtde_c.moveL(pose, speed, accel)

    def moveJ(self, q, speed=0.5, accel=0.5):
        self._connect_control()
        self.rtde_c.moveJ(q, speed=speed, acceleration=accel)

    def straighten_gripper(self, speed=0.8, accel=0.8):
        q = self.rtde_r.getActualQ()
        q[5] = 0.0
        self.moveJ(q, speed=speed, accel=accel)

    def freedrive_enable(self):
        self._connect_control()
        self.rtde_c.freedriveMode()
        time.sleep(0.5)

    def freedrive_disable(self):
        self._connect_control()
        self.rtde_c.endFreedriveMode()
        time.sleep(0.5)

    def calculate_offsets(self, char2cam):
        char2cam = np.array(char2cam)
        if char2cam.shape != (4, 4):
            raise ValueError("char2cam debe ser 4x4")
        self.straighten_gripper()
        self.freedrive_enable()
        print("Freedrive mode enabled")
        cam2base = np.array(self.config["HandEye_config"]["Cam2Base"])
        if cam2base.shape != (4, 4):
            raise ValueError("cam2base debe ser 4x4")
        try:
            print("\n--- OFFSET CALIBRATION ---")
            print("Keep same TCP orientation in all buttons (Critical)\n")
            for button in self.config["Buttons_offsets"].keys():
                input(f"Move the robot to button: {button} and press Enter...")
                butt2base = self._pose2matrix(self.rtde_r.getActualTCPPose())
                T_charuco_button = (self._matrixinv(char2cam) @ self._matrixinv(cam2base) @ butt2base)
                t = T_charuco_button[:3, 3]
                if np.linalg.norm(t) > 1:
                    print(f"⚠️ Weird offset in {button}: {t} (check frames or calibration)")
                self.config["Buttons_offsets"][button] = t.tolist()
                print(f"{button} offset: {t}")
        finally:
            self.freedrive_disable()
            print("Freedrive mode disabled")
        with open("config.json", "w") as f:
            json.dump(self.config, f, indent=2)
        print("Offsets guardados correctamente.")

    def _matrixinv(self, T):
        T = np.array(T)
        if T.shape != (4, 4):
            raise ValueError("La matriz debe ser 4x4")
        R = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return T_inv

    def _pose2matrix(self, pose):
        x, y, z, rx, ry, rz = pose
        rotvec = np.array([rx, ry, rz])
        R, _ = cv2.Rodrigues(rotvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def getActualTCPPose(self):
        return self.rtde_r.getActualTCPPose()

    def disconnect_all(self):
        if self.rtde_c is not None:
            try:
                self.rtde_c.disconnect()
                logging.info("RTDEControl desconectado.")
            except Exception as e:
                logging.warning(f"Error desconectando rtde_c: {e}")
            finally:
                self.rtde_c = None

        if self.rtde_r is not None:
            try:
                self.rtde_r.disconnect()
                logging.info("RTDEReceive desconectado.")
            except Exception as e:
                logging.warning(f"Error desconectando rtde_r: {e}")
            finally:
                self.rtde_r = None

        if self.rtde_io_iface is not None:
            try:
                self.rtde_io_iface.disconnect()
                logging.info("RTDE IO desconectado.")
            except Exception as e:
                logging.warning(f"Error desconectando rtde_io: {e}")
            finally:
                self.rtde_io_iface = None
                
    def write_float_register(self,reg,value):
        for attempt in range(5):
            if self.rtde_io_iface.setInputDoubleRegister(reg, value):
                break
            print(f"Fail in send data.... Retrying ({attempt+1}/5)")
        else:
            print("Error: No se pudo enviar el registro tras 5 intentos")
