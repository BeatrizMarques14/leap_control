#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "leap_control_node");  // Inicializa o nó
    ros::NodeHandle nh;  // Criar o manipulador do nó

    ROS_INFO("LEAP Control Node iniciado!");  // Mensagem de debug

    ros::spin();  // Mantém o nó ativo
    return 0;
}
