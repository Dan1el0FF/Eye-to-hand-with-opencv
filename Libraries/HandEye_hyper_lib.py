#This code only works on python 3.11.9 versions and some previous ones Charuco Version
#Dany_Libraries
import cv2
import numpy as np
import os
import json
from RobotClient_ultra_lib import RobotClient


DEFAULT_CONFIG = {
  "HandEye_config": {
    "foldername": "fotosNikon1080",
    "CHARUCO_SQUARES": [4, 5],
    "CHARUCO_SQUARE_SIZE": 0.045,
    "CHARUCO_MARKER_SIZE": 0.033,
    "REPROJ_FILTER_THRESH": 0.6,
    "Cam2Base": [
      [-0.97832385, -0.06457509, 0.19675491, -0.1763538],
      [-0.20629697, 0.38650931, -0.89891719, -0.06643018],
      [-0.01799995, -0.92002207, -0.39145292, 0.52965157],
      [0.0, 0.0, 0.0, 1.0]
    ]
  },
  "Transforms_config": {
    "CHARUCO_SQUARES": [7, 3],
    "CHARUCO_SQUARE_SIZE": 0.035,
    "CHARUCO_MARKER_SIZE": 0.026
  },
  "General_config": {
    "resolution": [1920, 1080],
    "cam_params": [2053.92452, 2056.73704, 959.799126, 510.913241],
    "dist_coeffs": [-0.398545281, -0.271468979, -0.00047782121, -0.00108854596, 1.59684202],
    "cam_index": 0,
    "ROBOT_IP": "192.168.0.3",
    "fps": 30,
    "streaming_fps": 25
  },
  "Buttons_offsets": {

    "Volume": [0.0,0.0,0.0],
    "Setting": [0.0,0.0,0.0],
    "Previous": [0.0,0.0,0.0],
    "PlayPause": [0.0,0.0,0.0],
    "Next": [0.0,0.0,0.0],
    "Clear": [0.0,0.0,0.0],
    "Tune": [0.0,0.0,0.0] 
  }
}

class Handeye:
    #constructor
    def __init__(self):

        if not os.path.exists("config.json"):
            with open("config.json", "w") as f:
                json.dump(DEFAULT_CONFIG, f, indent=2)
            print(f"Archivo config.json creado con valores por defecto, recuerda configurar con tus valores reales")
        with open("config.json", "r") as f:
            config = json.load(f)
        self.config = config
        print("Archivo config.jason encontrado, extrayendo parámetros...")
        self.folder = config["HandEye_config"]["foldername"]
        self.resolution = config["General_config"]["resolution"]
        self.ROBOT_IP = config["General_config"]["ROBOT_IP"]
        self.Cam_Index = config["General_config"]["cam_index"]
        self.UR_POSES = []
        self.REPROJ_FILTER_THRESH = config["HandEye_config"]["REPROJ_FILTER_THRESH"]
        Cam_Params = np.array(config["General_config"]["cam_params"])
        fx, fy, cx, cy = Cam_Params
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1]])
        self.dist_coeffs = np.array(config["General_config"]["dist_coeffs"])
        squares = config["HandEye_config"]["CHARUCO_SQUARES"]
        CHARUCO_SQUARES = (squares[0],squares[1])
        CHARUCO_SQUARE_SIZE = config["HandEye_config"]["CHARUCO_SQUARE_SIZE"]
        CHARUCO_MARKER_SIZE = config["HandEye_config"]["CHARUCO_MARKER_SIZE"]
        # CharucoBoard
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard(
            CHARUCO_SQUARES,
            CHARUCO_SQUARE_SIZE,
            CHARUCO_MARKER_SIZE,
            self.dictionary
        )
        self.detector_params  = cv2.aruco.DetectorParameters()
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)

    def start_eth_calib(self, robot):
        """Captura fotos y calcula Cam2Base.

        Args:
            robot: instancia de RobotClient inyectada desde el caller.
                   El freedrive y el disconnect son responsabilidad del caller.
        """
        print("Connect the camera and the UR to the computer and move it manually to the camera vision")
        os.makedirs(self.folder, exist_ok=True)

        try:
            test = robot.getActualTCPPose()
        except Exception as e:
            print(f"Error conectando con robot: {e}")
            return
        try:
            print(f"Succesfully connected to robot: {self.ROBOT_IP}\n Actual pose: {test}")
            cap = cv2.VideoCapture(self.Cam_Index, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Cámara lista en: {w}x{h}")
            if not cap.isOpened():
                print("No se pudo abrir la cámara")
                exit()

            contador = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("No se pudo capturar imagen")
                    break
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                charuco_corners, charuco_ids, marker_corners, marker_ids = \
                    self.charuco_detector.detectBoard(gray)

                preview = frame.copy()

                if marker_ids is not None and len(marker_ids) >= 3:
                    cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
                    if charuco_ids is not None and len(charuco_ids) >= 4:
                        cv2.aruco.drawDetectedCornersCharuco(preview, charuco_corners, charuco_ids)
                        cv2.putText(preview, f"Corners: {len(charuco_ids)} — OK para capturar",
                                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        cv2.putText(preview, "Pocos corners detectados",
                                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    cv2.putText(preview, "Board no detectado",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                scale = 0.7
                preview_resized = cv2.resize(
                    preview,
                    (0, 0),
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_AREA
                )

                cv2.imshow("Webcam (press space to take photo, press esc to finish)", preview_resized)

                key = cv2.waitKey(1) & 0xFF

                if key == 32:  # Espacio — capturar frame RAW (sin undistort) para procesamiento
                    nombre = f"{self.folder}/foto_{contador}.jpg"
                    cv2.imwrite(nombre, frame)
                    pose = robot.getActualTCPPose()
                    self.UR_POSES.append(pose)
                    print(f"Photo saved: {nombre}")
                    print(f"Pose saved:  {self.UR_POSES[contador]}")
                    contador += 1

                if key == 27:  # ESC — terminar
                    break

            cap.release()
            cv2.destroyAllWindows()

            print("\nPreparing data...")
            VR_gripper2base, t_gripper2base = self.separate_UrPoses()
            VR_target2cam, t_target2cam, indices_validos = self.charuco_data(contador)

            # Filtrar poses del UR que corresponden a fotos válidas
            VR_gripper2base = [VR_gripper2base[i] for i in indices_validos]
            t_gripper2base  = [t_gripper2base[i]  for i in indices_validos]

            R_gripper2base = self.normalize_rotations(VR_gripper2base)
            R_target2cam   = self.normalize_rotations(VR_target2cam)

            print(f"\nPoses válidas para hand-eye: {len(R_gripper2base)}")
            print("Calculando matriz homogenea cam2base...")

            R_cam2base, t_cam2base = self.calibrate_eye_hand(
                R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True
            )
            H_cam2base = np.eye(4)
            H_cam2base[:3, :3] = R_cam2base
            H_cam2base[:3, 3]  = t_cam2base.flatten()

            self.config["HandEye_config"]["Cam2Base"] = H_cam2base.tolist()

            with open("config.json", "w") as f:
                json.dump(self.config, f, indent=2)
            print("Cam2Base actualizado en config.json")
        finally:
            pass   # freedrive y disconnect son responsabilidad del caller
        return np.array(H_cam2base)

    def charuco_data(self, images):
        rvec_list      = []
        t_list         = []
        indices_validos = []
        reproj_errors  = []

        for i in range(images):
            nombre = f"foto_{i}.jpg"
            path = os.path.join(self.folder, nombre)
            print(f"\nProcesando: {nombre}")
            img = cv2.imread(path)
            if img is None:
                print("  No se pudo leer")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Detectar con nueva API (OpenCV 4.10+)
            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                self.charuco_detector.detectBoard(gray)

            if marker_ids is None or len(marker_ids) < 3:
                print(f"  No se detectó board (pocos markers: {0 if marker_ids is None else len(marker_ids)})")
                continue

            if charuco_ids is None or len(charuco_ids) < 4:
                print(f"  Insuficientes corners ChArUco: {0 if charuco_ids is None else len(charuco_ids)}")
                continue

            # Refinamiento subpíxel
            charuco_corners = cv2.cornerSubPix(
                gray, charuco_corners, (5, 5), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            # Estimar pose — imagen RAW → pasar dist_coeffs reales
            ret_pose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board,
                self.camera_matrix, self.dist_coeffs,
                None, None
            )
            if not ret_pose:
                print("  estimatePoseCharucoBoard falló")
                continue

            # Calcular error de reproyección para esta foto
            obj_pts, img_pts = self.board.matchImagePoints(charuco_corners, charuco_ids)
            projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            reproj_err = cv2.norm(img_pts, projected, cv2.NORM_L2) / len(projected)

            if reproj_err > self.REPROJ_FILTER_THRESH:
                print(f"  DESCARTADA — reproyección alta: {reproj_err:.4f} px (umbral: {self.REPROJ_FILTER_THRESH})")
                continue

            rvec_list.append(rvec.flatten())
            t_list.append(tvec.flatten())
            indices_validos.append(i)
            reproj_errors.append(reproj_err)
            print(f"  OK — {len(charuco_ids)} corners, reproyección: {reproj_err:.4f} px")

        if reproj_errors:
            print(f"\nReproyección promedio (fotos aceptadas): {np.mean(reproj_errors):.4f} px")
            print(f"Reproyección máxima: {np.max(reproj_errors):.4f} px")

        return rvec_list, t_list, indices_validos

    #Based on Torayeff method from opencv forum: https://forum.opencv.org/t/eye-to-hand-calibration/5690/2
    def calibrate_eye_hand(self, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True):
        if eye_to_hand:
            R_base2gripper, t_base2gripper = [], []
            for R, t in zip(R_gripper2base, t_gripper2base):
                R_b2g = R.T
                t_b2g = -R_b2g @ t
                R_base2gripper.append(R_b2g)
                t_base2gripper.append(t_b2g)
            R_g = np.array(R_base2gripper)
            t_g = np.array(t_base2gripper)
        else:
            R_g = np.array(R_gripper2base)
            t_g = np.array(t_gripper2base)

        methods = {
            "TSAI":      cv2.CALIB_HAND_EYE_TSAI,
            "PARK":      cv2.CALIB_HAND_EYE_PARK,
            "HORAUD":    cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF":   cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        print("\n--- Comparando métodos hand-eye ---")
        best_method = None
        best_R, best_t = None, None
        best_err = float("inf")
        for name, method in methods.items():
            try:
                R, t = cv2.calibrateHandEye(
                    R_gripper2base=R_g,
                    t_gripper2base=t_g,
                    R_target2cam=np.array(R_target2cam),
                    t_target2cam=np.array(t_target2cam),
                    method=method
                )
                # Métrica: consistencia de la solución — norma del residuo AX=XB
                err = self._handeye_residual(R_g, t_g, np.array(R_target2cam), np.array(t_target2cam), R, t)
                print(f"  {name:<12} residuo: {err:.6f}")
                if err < best_err:
                    best_err    = err
                    best_method = name
                    best_R, best_t = R, t
            except Exception as e:
                print(f"  {name:<12} falló: {e}")

        print(f"\n→ Mejor método: {best_method} (residuo: {best_err:.6f})")
        return best_R, best_t
    #This calculates the residual from the ecuation AX=BX
    def _handeye_residual(self, R_A, t_A, R_B, t_B, R_X, t_X):
        errors = []
        n = len(R_A)
        for i in range(n):
            for j in range(i + 1, n):
                # A = movimiento del gripper entre pose i y j
                A = np.eye(4)
                A[:3, :3] = R_A[j] @ R_A[i].T
                A[:3,  3] = (t_A[j] - R_A[j] @ R_A[i].T @ t_A[i]).flatten()
                # B = movimiento del board entre pose i y j
                B = np.eye(4)
                B[:3, :3] = R_B[j] @ R_B[i].T
                B[:3,  3] = (t_B[j] - R_B[j] @ R_B[i].T @ t_B[i]).flatten()
                # X = solución hand-eye
                X = np.eye(4)
                X[:3, :3] = R_X
                X[:3,  3] = t_X.flatten()
                # Residuo: AX - XB
                residuo = A @ X - X @ B
                errors.append(np.linalg.norm(residuo))
        return np.mean(errors)

    def separate_UrPoses(self):
        R_vecs, t_vecs = [], []
        for pose in self.UR_POSES:
            x, y, z, rx, ry, rz = pose
            R_vecs.append([rx, ry, rz])
            t_vecs.append([x, y, z])
        return R_vecs, t_vecs

    def normalize_rotations(self, r_vecs):
        matrix = []
        for rot in r_vecs:
            rot = np.array(rot).reshape(3, 1)
            R, _ = cv2.Rodrigues(rot)
            matrix.append(R)
        return matrix

    def show_UrPoses(self):
        print(f"\n--- Lista de {len(self.UR_POSES)} Poses Guardadas ---")
        for i, pose in enumerate(self.UR_POSES):
            print(f"Foto {i}: {pose}")


#Special thanks to Torayeff, Eduardo and crackwitz from opencv forum and also to ChThorn from github. Your work helped me a lot.
