#!/usr/bin/env python3

import rospy
from dynamixel_sdk import *  # Biblioteca Dynamixel SDK
from std_msgs.msg import Int32

# Configurações do Dynamixel
MY_DXL = "X_SERIES"
ADDR_PRESENT_POSITION = 132
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_ID = 3
DEVICENAME = '/dev/ttyUSB0'
DXL_MOVING_STATUS_THRESHOLD = 20

def read_position():
    """Função para ler e publicar a posição do motor"""
    rospy.init_node('read_position_node', anonymous=True)
    pub = rospy.Publisher('dynamixel_position', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Inicializar comunicação com o Dynamixel
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        rospy.logerr("Falha ao abrir a porta")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Falha ao configurar a baudrate")
        return

    rospy.loginfo("Dynamixel conectado!")

    while not rospy.is_shutdown():
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr(packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo(f"Posição Atual: {dxl_present_position}")
            pub.publish(dxl_present_position)

        rate.sleep()

    portHandler.closePort()

if __name__ == "__main__":
    try:
        read_position()
    except rospy.ROSInterruptException:
        pass
