Eye-to-Hand Calibration con OpenCV y UR5e

Sistema completo de calibración eye-to-hand para localizar objetos en el espacio de trabajo de un robot Universal Robots UR5e usando una cámara fija y tableros ChArUco. Una vez calibrado, el robot puede moverse a cualquier punto relativo a un marcador detectado por la cámara con una precisión de ±3.82 mm.

<img width="630" height="383" alt="image" src="https://github.com/user-attachments/assets/c6b13764-0c88-4f08-b5ea-193d785a7478" />

¿Qué es Eye-to-Hand?

En visión robótica, la calibración hand-eye consiste en encontrar la relación geométrica exacta entre una cámara y un robot. Sin ella, la cámara "ve" objetos en sus propias coordenadas (píxeles, metros respecto al lente), pero el robot no tiene idea de dónde están esos objetos respecto a su base. La calibración es el puente que traduce lo que ve la cámara a coordenadas que el robot entiende.

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

2. HandEye_hyper_lib.py — Calibración Eye-to-Hand

Responsabilidad: encontrar Cam2Base, la transformación que ubica la cámara respecto a la base del robot. Es el corazón del proyecto.

Con el tablero ChArUco sujeto al gripper, mueves el robot manualmente (en modo freedrive) a varias poses frente a la cámara. En cada captura, la librería guarda simultáneamente la foto (de donde saca tablero→cámara) y la pose actual del robot (de donde saca gripper→base). Con esos pares resuelve el problema AX = XB.


