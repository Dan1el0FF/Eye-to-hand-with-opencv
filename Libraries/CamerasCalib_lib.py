# Uso:
#   from CamerasCalib_lib import CamerasCalib
#   calib = CamerasCalib()
#   calib.run()                # captura fotos y calibra al presionar ESC
#   calib.capture_photos()     # solo captura
#   calib.calibrate()          # solo calibra con fotos ya existentes

import cv2
import glob
import json
import os

DEFAULT_CONFIG = {
    "HandEye_config": {
        "foldername": "eth_photos",
        "CHARUCO_SQUARES": [4, 5],
        "CHARUCO_SQUARE_SIZE": 0.045,
        "CHARUCO_MARKER_SIZE": 0.033,
        "REPROJ_FILTER_THRESH": 0.6,
        "Cam2Base": [
            [-0.97832385, -0.06457509, 0.19675491, -0.1763538],
            [-0.20629697,  0.38650931, -0.89891719, -0.06643018],
            [-0.01799995, -0.92002207, -0.39145292,  0.52965157],
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
    "CamerasCalib_config": {
        "foldername": "CamParams_photos"
    },
    "Buttons_offsets": {
        "Volume":    [0.0, 0.0, 0.0],
        "Settings":  [0.0, 0.0, 0.0],
        "Previous":  [0.0, 0.0, 0.0],
        "PlayPause": [0.0, 0.0, 0.0],
        "Next":      [0.0, 0.0, 0.0],
        "Clear":     [0.0, 0.0, 0.0],
        "Tune":      [0.0, 0.0, 0.0]
    }
}


class CamerasCalib:

    def __init__(self):
        if not os.path.exists("config.json"):
            with open("config.json", "w") as f:
                json.dump(DEFAULT_CONFIG, f, indent=2)
            print("config.json no encontrado — creado con valores por defecto. Configura tus valores reales.")

        with open("config.json", "r") as f:
            self.config = json.load(f)
        print("config.json encontrado, extrayendo parámetros...")

        # General
        general = self.config["General_config"]
        self.cam_index  = general["cam_index"]
        self.resolution = general["resolution"]

        # HandEye (board + parámetros de calibración)
        handeye = self.config["HandEye_config"]
        self.reproj_thresh = handeye["REPROJ_FILTER_THRESH"]

        # Carpeta exclusiva para fotos de calibración de cámara
        self.folder = self.config["CamerasCalib_config"]["foldername"]

        squares = handeye["CHARUCO_SQUARES"]
        self.charuco_squares     = (squares[0], squares[1])
        self.charuco_square_size = handeye["CHARUCO_SQUARE_SIZE"]
        self.charuco_marker_size = handeye["CHARUCO_MARKER_SIZE"]

        # Board y detectores ChArUco
        self.dictionary       = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board            = cv2.aruco.CharucoBoard(
            self.charuco_squares,
            self.charuco_square_size,
            self.charuco_marker_size,
            self.dictionary
        )
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)

        self.min_corners = 6        # mínimo de corners ChArUco por imagen
        self.use_rational_model = False  # True = 8 coefs, False = 5 (estándar)

    # ──────────────────────────────────────────
    # FLUJO COMPLETO: captura + calibración
    # ──────────────────────────────────────────
    def run(self):
        """Captura fotos y al presionar ESC calibra automáticamente y guarda en config.json."""
        self.capture_photos()
        self.calibrate()

    def capture_photos(self):
        """Abre la cámara. ESPACIO guarda foto, ESC termina la captura."""
        os.makedirs(self.folder, exist_ok=True)

        cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Cámara lista en: {w}x{h}")
        print("ESPACIO → guardar foto   |   ESC → terminar captura\n")

        if not cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara")

        contador = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("No se pudo capturar imagen")
                break

            # Detección en tiempo real para feedback visual
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                self.charuco_detector.detectBoard(gray)

            preview = frame.copy()

            # Dibujar SIEMPRE en resolución original
            if marker_ids is not None and len(marker_ids) >= 3:
                cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
                if charuco_ids is not None and len(charuco_ids) >= self.min_corners:
                    cv2.aruco.drawDetectedCornersCharuco(preview, charuco_corners, charuco_ids)
                    cv2.putText(preview,
                                f"Fotos: {contador}  |  Corners: {len(charuco_ids)} — OK para capturar",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    cv2.putText(preview,
                                f"Fotos: {contador}  |  Pocos corners detectados",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(preview,
                            f"Fotos: {contador}  |  Board no detectado",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
            scale = 0.7  # cámbialo a 0.8 si quieres 80%
            preview_resized = cv2.resize(
                preview,
                (0, 0),
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA
            )

            cv2.imshow("CamerasCalib — ESPACIO=foto  ESC=calibrar", preview_resized)

            key = cv2.waitKey(1) & 0xFF

            if key == 32:   # ESPACIO — guardar foto RAW
                nombre = os.path.join(self.folder, f"foto_{contador:03d}.jpg")
                cv2.imwrite(nombre, frame)
                print(f"  Foto guardada: {nombre}")
                contador += 1

            elif key == 27:  # ESC
                break

        cap.release()
        cv2.destroyAllWindows()
        print(f"\nCaptura finalizada — {contador} fotos guardadas en '{self.folder}/'")

    # ──────────────────────────────────────────
    # FASE 2: CALIBRACIÓN
    # ──────────────────────────────────────────
    def calibrate(self):
        """
        Calibra con las imágenes de self.folder, actualiza cam_params y
        dist_coeffs en config.json y devuelve (camera_matrix, dist_coeffs).
        """
        image_glob = os.path.join(self.folder, "*.jpg")
        images = glob.glob(image_glob)

        if not images:
            raise FileNotFoundError(f"No se encontraron imágenes en {image_glob}")

        print(f"\n{'='*50}")
        print(f"CALIBRACIÓN ChArUco")
        print(f"  Board : {self.charuco_squares[0]}×{self.charuco_squares[1]} cuadros")
        print(f"  Cuadro: {self.charuco_square_size*1000:.1f} mm  Marker: {self.charuco_marker_size*1000:.1f} mm")
        print(f"  Umbral reproyección: {self.reproj_thresh} px")
        print(f"{'='*50}\n")
        print(f"Procesando {len(images)} imágenes...\n")

        # ── Detección de corners ──────────────
        all_corners, all_ids = [], []
        valid_images = []
        image_size = None

        for fname in sorted(images):
            img = cv2.imread(fname)
            if img is None:
                print(f"  {fname} ✖ no se pudo leer")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if image_size is None:
                image_size = gray.shape[::-1]   # (width, height)

            charuco_corners, charuco_ids, marker_corners, marker_ids = \
                self.charuco_detector.detectBoard(gray)

            if marker_ids is None or len(marker_ids) < 3:
                n = 0 if marker_ids is None else len(marker_ids)
                print(f"  {fname} ✖ muy pocos markers ({n})")
                continue

            if charuco_ids is None or len(charuco_ids) < self.min_corners:
                n = 0 if charuco_ids is None else len(charuco_ids)
                print(f"  {fname} ✖ solo {n} corners ChArUco (mínimo {self.min_corners})")
                continue

            charuco_corners = cv2.cornerSubPix(
                gray, charuco_corners, (5, 5), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
            valid_images.append(fname)
            print(f"  {fname} ✔  {len(charuco_ids)} corners")

        n_valid = len(valid_images)
        print(f"\nImágenes válidas: {n_valid} / {len(images)}")

        if n_valid < 10:
            print("⚠ Pocas imágenes válidas — se recomienda mínimo 10-15")
        if n_valid == 0:
            raise RuntimeError("No hay imágenes válidas para calibrar.")

        # ── Primera calibración ───────────────
        flags = cv2.CALIB_RATIONAL_MODEL if self.use_rational_model else 0

        def _run_calib(corners, ids):
            obj_pts = [self.board.matchImagePoints(c, i)[0] for c, i in zip(corners, ids)]
            img_pts = [self.board.matchImagePoints(c, i)[1] for c, i in zip(corners, ids)]
            return cv2.calibrateCameraExtended(
                obj_pts, img_pts, image_size, None, None, flags=flags
            )[:5]

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = _run_calib(all_corners, all_ids)

        # ── Error por imagen + filtro de outliers ──
        print("\n--- Error de reproyección por imagen ---")
        errors = []
        for i in range(n_valid):
            obj_pts, img_pts = self.board.matchImagePoints(all_corners[i], all_ids[i])
            projected, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            err = cv2.norm(img_pts, projected, cv2.NORM_L2) / len(projected)
            errors.append(err)
            tag = "✔" if err <= self.reproj_thresh else "⚠ OUTLIER"
            print(f"  {valid_images[i]}: {err:.4f} px  {tag}")

        idx_ok  = [i for i, e in enumerate(errors) if e <= self.reproj_thresh]
        idx_out = [i for i, e in enumerate(errors) if e >  self.reproj_thresh]

        if idx_out:
            print(f"\nDescartando {len(idx_out)} imagen(es) con error > {self.reproj_thresh} px y recalibrando...")
            corners_f = [all_corners[i] for i in idx_ok]
            ids_f     = [all_ids[i]     for i in idx_ok]
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = _run_calib(corners_f, ids_f)
            print("Recalibración completa.")
        else:
            print("\nNingún outlier — no se necesita recalibrar.")
            corners_f, ids_f = all_corners, all_ids

        # ── Error final promedio ──────────────
        total_err = 0.0
        n_final = len(rvecs)
        for i in range(n_final):
            obj_pts, img_pts = self.board.matchImagePoints(corners_f[i], ids_f[i])
            projected, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            total_err += cv2.norm(img_pts, projected, cv2.NORM_L2) / len(projected)
        mean_error = total_err / n_final

        # ── Resultados ────────────────────────
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        print("\n" + "="*50)
        print("MATRIZ INTRÍNSECA (Camera Matrix):")
        print(camera_matrix)
        print("\nCOEFICIENTES DE DISTORSIÓN:")
        print(dist_coeffs)
        print(f"\ncam_params  = [{fx:.5f}, {fy:.5f}, {cx:.5f}, {cy:.5f}]")
        print(f"dist_coeffs = {dist_coeffs.flatten().tolist()}")
        print(f"\nERROR PROMEDIO FINAL: {mean_error:.4f} px")
        print(f"Imágenes usadas: {n_final}  |  Descartadas: {len(idx_out)}")

        if mean_error < 0.3:
            print("🔥 Excelente calibración (nivel industrial)")
        elif mean_error < 0.5:
            print("👍 Muy buena calibración")
        elif mean_error < 1.0:
            print("⚠ Aceptable pero puede mejorar")
        else:
            print("❌ Mala calibración — revisa las fotos y el tamaño del board")

        # ── Guardar en config.json ────────────
        self.config["General_config"]["cam_params"]  = [fx, fy, cx, cy]
        self.config["General_config"]["dist_coeffs"] = dist_coeffs.flatten().tolist()

        with open("config.json", "w") as f:
            json.dump(self.config, f, indent=2)
        print("\ncam_params y dist_coeffs actualizados en config.json")

        # Actualizar atributos de instancia por si se reutiliza el objeto
        self.camera_matrix = camera_matrix
        self.dist_coeffs   = dist_coeffs

        return camera_matrix, dist_coeffs
