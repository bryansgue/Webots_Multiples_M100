import os
import sys
import math
from controller import Supervisor
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

MAXIMUM_NUMBER_OF_COORDINATES = 50

# Variable global para almacenar la matriz recibida
received_matrix = None
num_cols = None

# Función de devolución de llamada para procesar mensajes Float32MultiArray
def prediction_callback(msg):
    # Almacena la matriz del mensaje en la variable global
    global received_matrix
    global num_cols
    data = np.array(msg.data)
    num_cols = len(data) // 3  # Calcula la cantidad de columnas automáticamente
    received_matrix = data.reshape((3, num_cols))

# Función principal de Webots
def main():
    # Inicializa el nodo ROS en Webots
    rospy.init_node('webots_node', anonymous=True)

    # Suscribirse al tópico /Prediction y llamar a prediction_callback
    rospy.Subscriber('/Prediction_2', Float64MultiArray, prediction_callback)

    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    # Obtener referencias importantes a los subnodos de LINE.
    line_set_node = supervisor.getFromDef("LINE_SET_2")
    coordinates_node = line_set_node.getField("coord").getSFNode()
    point_field = coordinates_node.getField("point")
    coord_index_field = line_set_node.getField("coordIndex")

    # Matriz de puntos 3xN (inicialmente vacía)
    points = []
    num_points = None
    
    
    # Inicializa todos los puntos en cero fuera del bucle principal
    for i in range(point_field.getCount()):
        point_field.setMFVec3f(i, [0, 0, 0])
    
    while num_points is None:
        num_points = num_cols  # Actualiza num_points con el valor de num_cols
        rospy.sleep(0.1)  # Espera 0.1 segundos antes de verificar nuevamente

    # Bucle principal
    while supervisor.step(time_step) != -1:
        # Actualiza la matriz de puntos con received_matrix si está disponible
        if received_matrix is not None:
            points = received_matrix.T.tolist()  # Transpone y convierte en lista
            print(num_points)
        # Establecer los puntos en el campo 'point' uno por uno
        for i in range(num_points):
            if i < len(points):
                point_field.setMFVec3f(i, [points[i][0], points[i][1], points[i][2]])
        
        
        # Actualizar el campo 'coordIndex' para conectar todos los puntos en orden
        coord_index = []
        for i in range(num_points):
            coord_index.extend([i, (i + 1) % num_points, -1])  # Conectar puntos en orden

        # Evitar que el último punto se conecte al primero
        coord_index[-3] = -1

        # Establecer los valores del campo 'coordIndex'
        for i, value in enumerate(coord_index):
            coord_index_field.setMFInt32(i, value)

    print("ROS Node Shutdown")

if __name__ == '__main__':
    main()