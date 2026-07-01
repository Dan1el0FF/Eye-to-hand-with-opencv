Eye-to-Hand Calibration con OpenCV y UR5e

Sistema completo de calibración eye-to-hand para localizar objetos en el espacio de trabajo de un robot Universal Robots UR5e usando una cámara fija y tableros ChArUco. Una vez calibrado, el robot puede moverse a cualquier punto relativo a un marcador detectado por la cámara con una precisión de ± 3.82 mm. Este proyecto fue creado para una empresa de vehiculos con el objetivo de encontrar fallas en tableros automotrices antes de entrar al mercado.


<img width="630" height="383" alt="image" src="https://github.com/user-attachments/assets/c6b13764-0c88-4f08-b5ea-193d785a7478" />

¿Qué es Eye-to-Hand?

En visión robótica, la calibración hand-eye consiste en encontrar la relación geométrica exacta entre una cámara y un robot. Sin ella, la cámara "ve" objetos en sus propias coordenadas (píxeles, metros respecto al lente), pero el robot no tiene idea de dónde están esos objetos respecto a la base del robot. La calibración es el puente que traduce lo que ve la cámara a coordenadas que el robot entiende.

Existen dos configuraciones posibles:


Eye-in-hand (ojo en la mano): la cámara va montada en la muñeca del robot y se mueve junto con él.
Eye-to-hand (ojo hacia la mano): la cámara está fija en el entorno, observando desde fuera al robot y a su zona de trabajo. Esta es la configuración que implementa este repositorio.


En eye-to-hand, la incógnita que resolvemos es la transformación rígida entre el marco de la cámara y el marco de la base del robot, representada por una matriz homogénea 4×4 llamada en este proyecto Cam2Base. Una vez que conocemos esa matriz, todo lo que la cámara detecte puede convertirse a coordenadas de la base del robot, y por tanto el robot puede alcanzarlo.


En este proyecto en particular, la calibración se usa para que el UR5e pueda posicionar su gripper sobre puntos definidos respecto a un tablero ChArUco de referencia (de hecho, el config.json incluye una sección Buttons_offsets pensada para presionar botones físicos en posiciones relativas al tablero).

Las matemáticas en una cáscara de nuez

La calibración eye-to-hand se reduce a resolver un problema clásico conocido como AX = XB:


Se fija un tablero de calibración (ChArUco) al efector final del robot.
Se mueve el robot a N poses distintas dentro del campo de visión de la cámara.
En cada pose se registran dos datos:

Gripper → Base: la pose del efector respecto a la base, leída directamente de los encoders del robot (cinemática directa).
Tablero → Cámara: la pose del tablero respecto a la cámara, obtenida detectando el ChArUco en la imagen.



Con varias parejas de estos datos se resuelve el sistema y se obtiene la transformación fija buscada.


OpenCV ofrece cinco métodos para resolver este sistema (Tsai-Lenz, Park, Horaud, Andreff y Daniilidis). Este proyecto los prueba todos automáticamente y se queda con el de menor residuo, lo cual es una mejora notable sobre implementaciones que usan un solo método a ciegas.


Para correr el código se debe tener la version de python 3.11.9 ya que es compatible con la libreria llamada rtde (real time data exchange) para la comunicación con el robot UR5e.

Probé la presición utilizando un chessboard vs un ChAruco y la diferencia fue avismal por eso mismo todo el código utilza ChArucos los cuales puedes encontrar en el repositorio.

Imágen de uno de los Charucos utilizados:


<img width="235" height="281" alt="image" src="https://github.com/user-attachments/assets/b0afcfe1-9c45-4e0c-be9a-ff13b6b19f04" />


¿Cómo funciona este proyecto?

1.- Calibración intrínseca → encuentra los parámetros internos de la cámara (distancia focal, centro óptico) y los coeficientes de distorsión del lente. Sin esto, las mediciones de profundidad están mal.

2.- Calibración Eye-to-Hand → encuentra Cam2Base, la posición de la cámara respecto a la base del robot.

3.- Localización → con todo calibrado, la cámara detecta un tablero ChArUco de referencia en la mesa y el robot se mueve a posiciones relativas a él.

Detalle importante: el proyecto usa dos tableros ChArUco diferentes. Uno (4×5, cuadros de 45 mm) se fija al gripper durante la calibración hand-eye. El otro (7×3, cuadros de 35 mm) se coloca en la mesa como marco de referencia de los objetos durante la operación. Cada uno tiene su propia sección en config.json.


Requisitos:

Para correr correctamente todos los archivos deben estar a la misma altura.

Software

Python 3.11.9 (obligatorio). Esta versión específica es necesaria por compatibilidad con la librería rtde (Real-Time Data Exchange) que comunica con el UR5e.
Dependencias de Python:

pip install opencv-contrib-python numpy ur-rtde


Hardware


Robot Universal Robots UR5e (con su controlador en red).
Una cámara fija (webcam o cámara industrial) apuntando a la zona de trabajo.
Tableros de calibración ChArUco impresos con las dimensiones exactas configuradas en config.json.
El robot y la PC en la misma red. La IP del robot se configura en config.json (ROBOT_IP, por defecto 192.168.0.3).

Las librerías

El sistema está dividido en cuatro librerías, cada una con una responsabilidad clara. Tres de ellas implementan una etapa del pipeline, y la cuarta (RobotClient) es la capa de comunicación que todas comparten.

1. CamerasCalib_lib.py — Calibración intrínseca

Responsabilidad: encontrar los parámetros internos de la cámara y la distorsión del lente.

Esta librería abre la cámara, te deja capturar fotos de un tablero ChArUco desde distintos ángulos, y calcula la matriz intrínseca (fx, fy, cx, cy) y los coeficientes de distorsión. Estos valores describen cómo la cámara proyecta el mundo 3D en píxeles 2D, y son la base de toda medición posterior.

Características destacadas:

Refinamiento subpíxel (cornerSubPix) para detectar las esquinas con máxima precisión.
Filtro automático de outliers: calcula el error de reproyección de cada foto y descarta las que superan un umbral (REPROJ_FILTER_THRESH), luego recalibra solo con las buenas.
Diagnóstico de calidad: clasifica el resultado final (🔥 industrial < 0.3 px, 👍 muy buena < 0.5 px, ⚠ aceptable < 1.0 px, ❌ mala).

2. HandEye_hyper_lib.py — Calibración Eye-to-Hand

Responsabilidad: encontrar Cam2Base, la transformación que ubica la cámara respecto a la base del robot. Es el corazón del proyecto.

Con el tablero ChArUco sujeto al gripper, mueves el robot manualmente (en modo freedrive) a varias poses frente a la cámara. En cada captura, la librería guarda simultáneamente la foto (de donde saca tablero→cámara) y la pose actual del robot (de donde saca gripper→base). Con esos pares resuelve el problema AX = XB.

Características destacadas:

Comparación automática de métodos: Tsai, Park, Horaud, Andreff y Daniilidis se prueban todos; gana el de menor error. Esto evita el típico error de confiar ciegamente en un solo método.
Truco eye-to-hand: como cv2.calibrateHandEye() está pensado originalmente para eye-in-hand, cuando eye_to_hand=True la librería invierte las poses gripper→base a base→gripper antes de resolver. Este es el detalle clave que adapta el algoritmo a la configuración de cámara fija.
Inyección de dependencias: recibe la instancia de robot desde afuera, dejando que quien la llama controle el freedrive y la desconexión.

3.- Transforms_hyper_lib.py — Transformaciones y localización

Responsabilidad: una vez calibrado todo, detectar el tablero de referencia y convertir su posición a coordenadas que el robot pueda alcanzar.

Esta es la librería que usas en operación normal. Detecta el tablero ChArUco de la mesa, calcula dónde está respecto a la cámara, y encadena las transformaciones para obtener su posición respecto a la base del robot.

Características destacadas:

Promediado robusto de poses: en vez de una sola lectura, acumula muchas muestras, filtra las traslaciones atípicas (a más de 2σ de la media) y promedia las rotaciones correctamente vía SVD (no se puede promediar matrices de rotación sumándolas sin más; el código las re-ortogonaliza).

4. RobotClient_ultra_lib.py — Comunicación con el UR5e

Responsabilidad: toda la comunicación con el robot. Es la capa que todas las demás librerías usan para mover el brazo, leer su pose y correr programas.

Combina dos canales de comunicación del UR: la interfaz RTDE (para movimientos y lectura de datos en tiempo real) y la interfaz Dashboard vía socket (para cargar y ejecutar programas .urp). Gestiona la coexistencia de ambos canales, que es históricamente delicada.

Características destacadas:

Gestión cuidadosa de RTDE vs Dashboard: los dos canales no pueden usar el socket al mismo tiempo, así que la librería desconecta RTDE antes de mandar comandos al Dashboard y maneja los tiempos de espera (sleep) críticos que el controlador del UR necesita para no fallar. Los comentarios # FIX: documentan cada uno de estos detalles aprendidos a base de prueba y error.
Helpers de álgebra lineal: _pose2matrix() y _matrixinv() para convertir e invertir transformaciones homogéneas.

Créditos y referencias

Este proyecto se apoya en el trabajo y la ayuda de varias personas de la comunidad:


El método de calibración eye-to-hand está basado en el aporte de Torayeff en el foro de OpenCV: Eye-to-hand calibration.
Agradecimientos especiales a Torayeff, Eduardo y crackwitz del foro de OpenCV, y a ChThorn en GitHub, cuyo trabajo fue de gran ayuda.
La librería RobotClient está parcialmente basada en el trabajo de Mariana.
