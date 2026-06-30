#Mejora en el promedio de fotos y configuracion en json
#Dany_Libraries
import cv2
import numpy as np
import os
import json
import rtde_control
import rtde_receive

class Transforms:

    def __init__(self):
        
        if not os.path.exists("config.json"):
            print("Archivo config.json no encontrado, Corre HandEye_hyper_lib para crear el archivo..")
            exit()
        with open("config.json", "r") as f:
            config = json.load(f)
        self.cam2base = np.array(config["HandEye_config"]["Cam2Base"])
        self.dist_coeffs = np.array(config["General_config"]["dist_coeffs"])
        self.cam_params = np.array(config["General_config"]["cam_params"])
        self.cam_index = config["General_config"]["cam_index"]
        self.resolution = config["General_config"]["resolution"]
        fx, fy, cx, cy = self.cam_params
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1]])
        squares = config["Transforms_config"]["CHARUCO_SQUARES"]
        CHARUCO_SQUARES = (squares[0],squares[1])
        CHARUCO_SQUARE_SIZE = config["Transforms_config"]["CHARUCO_SQUARE_SIZE"]
        CHARUCO_MARKER_SIZE = config["Transforms_config"]["CHARUCO_MARKER_SIZE"]
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard(
            CHARUCO_SQUARES,
            CHARUCO_SQUARE_SIZE,
            CHARUCO_MARKER_SIZE,
            self.dictionary
        )
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)
        self.ROBOT_IP = config["General_config"]["ROBOT_IP"]
        self.start_camera()

    
    def detect_tag2cam(self, max_attempts=150, n_samples=20):
        rvecs_accum = []
        tvecs_accum = []

        # Flush del buffer antes de empezar para descartar frames viejos
        for _ in range(5):
            self.cap.read()

        attempts = 0
        while len(rvecs_accum) < n_samples and attempts < max_attempts:
            attempts += 1
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                self.charuco_detector.detectBoard(gray)

            if marker_ids is None or len(marker_ids) < 3:
                continue
            if charuco_ids is None or len(charuco_ids) < 4:
                continue

            ret_pose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board,
                self.camera_matrix, self.dist_coeffs,
                None, None
            )
            if not ret_pose:
                continue

            rvecs_accum.append(rvec.flatten())
            tvecs_accum.append(tvec.flatten())

        if len(rvecs_accum) == 0:
            print("Error: No CharucoBoard detected")
            self.cap.release()
            return None, False

        # =========================
        # 🔹 FILTRO DE OUTLIERS EN TRASLACIÓN
        # =========================
        t_array = np.array(tvecs_accum)
        t_mean = np.mean(t_array, axis=0)
        t_std  = np.std(t_array, axis=0)

        t_filtered = [t for t in t_array if np.all(np.abs(t - t_mean) < 2 * t_std)]
        t_filtered = np.array(t_filtered)

        t_mean = np.mean(t_filtered, axis=0)

        # =========================
        # 🔹 PROMEDIO CORRECTO DE ROTACIONES (SVD)
        # =========================
        R_list = [cv2.Rodrigues(r.reshape(3, 1))[0] for r in rvecs_accum]

        R_sum = np.zeros((3, 3))
        for R in R_list:
            R_sum += R
        R_avg = R_sum / len(R_list)

        # Re-ortogonalizar
        U, _, Vt = np.linalg.svd(R_avg)
        R_mean = U @ Vt
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1
            R_mean = U @ Vt

        T = np.eye(4)
        T[:3, :3] = R_mean
        T[:3, 3]  = t_mean

        return T, True

    def get_tag2base(self, tag2cam, offset=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0]):
        
        def rot_x(rx):
            return np.array([
                [1, 0, 0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx),  np.cos(rx)]
            ])

        def rot_y(ry):
            return np.array([
                [ np.cos(ry), 0, np.sin(ry)],
                [0, 1, 0],
                [-np.sin(ry), 0, np.cos(ry)]
            ])

        def rot_z(rz):
            return np.array([
                [np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz),  np.cos(rz), 0],
                [0, 0, 1]
            ])
        rx, ry, rz = rotation  # en radianes

        # Matriz de rotación (orden ZYX típico en robótica)
        R = rot_z(rz) @ rot_y(ry) @ rot_x(rx)

        H_offset = np.eye(4)
        H_offset[:3, :3] = R
        H_offset[:3, 3] = offset

        tag2base = self.cam2base @ tag2cam @ H_offset
        return tag2base

    def matrix2pose(self, H_matrix):
        H_matrix = np.array(H_matrix)
        x = H_matrix[0, 3]
        y = H_matrix[1, 3]
        z = H_matrix[2, 3]
        R = H_matrix[:3, :3]
        rvec, _ = cv2.Rodrigues(R)
        rx = rvec[0][0]
        ry = rvec[1][0]
        rz = rvec[2][0]
        return [x, y, z, rx, ry, rz]

    def pose2matrix(self, pose):
        x, y, z, rx, ry, rz = pose
        rotvec = np.array([rx, ry, rz])
        R, _ = cv2.Rodrigues(rotvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def start_camera(self):
        self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        if not self.cap.isOpened():
            raise RuntimeError("Error to open camera")
        for _ in range(20):
            self.cap.read()
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Camera ready in {int(w)}x{int(h)} pixels config")

    def stop_camera(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
