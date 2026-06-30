from Libraries.CamerasCalib_lib import CamerasCalib
from Libraries.HandEye_hyper_lib import Handeye
from Libraries.Transforms_hyper_lib import Transforms
from Libraries.RobotClient_ultra_lib import RobotClient

Cam = CamerasCalib()
eth = Handeye()
robot = RobotClient()
tf = Transforms()


Home_foto_q = [1.6192622184753418, -1.7502428493895472,
               2.477604691182272, -0.7418721479228516,
               1.6633427143096924, -0.0006693045245569351]   #posición articular en radianes de cada parte del robot para moveJ

#Primero debemos encontrar los parámetros de la cámara es decir los parámetros intrínsecos y los de distorción:
#Para ello utilizaremos la libreria CameraCalib en donde es muy importante tener configurado los siguientes parámetros del archivo config.json:
#resolution, camera index, Camera folder name, Charuco squares, Square_Size, Marker_Size, los demás parámetros los podemos dejar como están por defecto.

#si queremos tomar las fotos desde cero ponemos true, pero si ya tenemos una carpeta con fotos por defecto pondremos false y en config.json tendremos que
#poner el nombre de la carpeta donde tenemos las fotos guardadas. (Cuidado por que si pones tomar nuevas fotos se van a borrar las fotos que existen en la carpeta seleccionada en config.json)

r = str(input("Desea Calibrar los parametros de la camara? (y/n)")).lower()

if(r == "y"):
    New_photos = True #cambiar manualmente
    if(New_photos): #toma fotos nuevas y borra las fotos de la carpeta actual
        Cam.run()
    else:
        Cam.calibrate() #calibra con fotos tomadas previamente

#Se realizara un filtro para descartar fotos de mala calidad y una vez finalizado el proceso se guardarán los parámetros nuevos en config.json y también se imprimen en la consola

#Ahora sigue el paso de obtener la matriz homogenea cam2base que quiere decir donde se encuentra la cámara respecto a la base del robot (UR5E)


#Recuerda configurar correctamente todos los parámetros de config.json los mas importantes son el tamaño del charuco, tamaño de los marcadores, cuantos cuadrados son, resolución
#camera index, parametros intrinsecos y de distorción deben ser correctos, ip del robot por que es de donde vamos a tomar los datos de la pose actual y finalmente el nombre
#del folder donde se guardaran las fotos o tambien lo puedes dejar por defecto como esta.

r = str(input("Desea calibrar eye to hand? (y/n)")).lower()
if(r == "y"):
    eth.start_eth_calib(robot)

#al finalizar imprime en terminal cam2base y tambien lo guarda como parametro en config.json ahora con toda esta calibración lista ya podemos empezar a localizar objetos con una
#presicion de 3.82 mm para lo cual vamos a necesitar realizar unas transformaciones de matrices asi que dare un ejemplo simple de como hacerlo utilzando la libreria Transforms

robot.moveJ(Home_foto_q) #movemos el robot a una pose en donde no estorbe a la cámara para tomar fotos

tag2cam,ok = tf.detect_tag2cam() #detectamos a que distancia se encuentra el charuco respecto a la cámara, es decir tag2cam.

if(ok): #si se detecta el charuco:
    tag2base = tf.get_tag2base(tag2cam,[0.0, 0.105, 0.0], [0, 0, 3.1416]) #transformamos tag2cam a tag2base y agregamos un offset relativo de 10.5cm en y junto
    #con una rotación en z de 180 grados, debemos poner los parametros en metros y radianes.
    pose = tf.matrix2pose(tag2base) #para mover el robot debemos transformar la matriz homogenea en una pose de rodriguez que el Ur5e pueda entender.
    robot.moveL(pose) #movemos el robot al (0,0,0) del charuco con un desplazamiento hacia abajo de 10.5 cm y con una rotación del gripper de 180 grados.
else: #si no se detecta el charuco:
    print("No se detectó charuco")

#Listo con este código podemos mover el robot a cualquier posición relativa al charuco con una presición de 3.82 mm







