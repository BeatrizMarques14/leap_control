#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK

# Configuração do motor
DXL_ID = 3  # ID do motor (altere conforme necessário)
BAUDRATE = 57600
DEVICENAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
TORQUE_ENABLE = 64  

# Endereços para Indirect Address
ADDR_INDIRECT_START = 168
ADDR_INDIRECT_DATA_START = 208  # Local onde os dados mapeados aparecem

# Endereços reais dos dados que queremos mapear
ADDR_PRESENT_POSITION = 132  # 4 bytes
ADDR_PRESENT_VELOCITY = 128  # 4 bytes
ADDR_PRESENT_PWM = 124

ADDR_GOAL_POSITION = 116

# Inicializa a comunicação
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def setup_dynamixel():
    """ Configura os Indirect Addresses para ler posição e velocidade. """
    if not portHandler.openPort():
        rospy.logerr("Erro ao abrir a porta!")
        return False

    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Erro ao configurar a taxa de baud!")
        return False

    # Desativa o torque antes da configuração
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, TORQUE_ENABLE, 0)

    # Configura Indirect Address para Posição
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START, ADDR_PRESENT_POSITION)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 2, ADDR_PRESENT_POSITION + 1)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 4, ADDR_PRESENT_POSITION + 2)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 6, ADDR_PRESENT_POSITION + 3)

    # Configura Indirect Address para Velocidade
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 8, ADDR_PRESENT_VELOCITY)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 10, ADDR_PRESENT_VELOCITY + 1)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 12, ADDR_PRESENT_VELOCITY + 2)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 14, ADDR_PRESENT_VELOCITY + 3)

    # Configura Indirect Address para PWM
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 16, ADDR_PRESENT_PWM)
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_INDIRECT_START + 18, ADDR_PRESENT_PWM + 1)

    # Ativa o torque novamente
    #packetHandler.write1ByteTxRx(portHandler, DXL_ID, TORQUE_ENABLE, 1)

    return True

def set_motor_position(msg):
    """ Callback que recebe a posição desejada e envia para o motor Dynamixel. """
    position = msg.data
 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr(f"Falha ao enviar posição! Código de erro: {dxl_comm_result}")
    elif dxl_error != 0:
        rospy.logerr(f"Erro de status do motor: {dxl_error}")

def read_dynamixel():
    """ Lê posição e velocidade do motor usando Indirect Data e publica em tópicos ROS. """
    rospy.init_node('dynamixel_reader', anonymous=True)
    pos_pub = rospy.Publisher('/dynamixel/position', Int32, queue_size=10)
    vel_pub = rospy.Publisher('/dynamixel/velocity', Int32, queue_size=10)
    pwm_pub = rospy.Publisher('/dynamixel/pwm', Int32, queue_size=10)

    # Subscriber para receber comandos de posição
    rospy.Subscriber('/set_position', Int32, set_motor_position)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        dxl_data, dxl_comm_result, dxl_error = packetHandler.readTxRx(portHandler, DXL_ID, ADDR_INDIRECT_DATA_START, 10)

        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            pos = int.from_bytes(dxl_data[0:4], byteorder='little', signed=True)
            vel = int.from_bytes(dxl_data[4:8], byteorder='little', signed=True)
            pwm = int.from_bytes(dxl_data[8:10], byteorder='little', signed=True)

            pos_pub.publish(pos)
            vel_pub.publish(vel)
            pwm_pub.publish(pwm)
        else:
            rospy.logwarn(f"Erro na leitura: {dxl_comm_result}, {dxl_error}")

        rate.sleep()

    portHandler.closePort()

if __name__ == '__main__':
    try:
        if setup_dynamixel():
            read_dynamixel()
    except rospy.ROSInterruptException:
        portHandler.closePort()
        rospy.loginfo("Encerrando nó Dynamixel")
