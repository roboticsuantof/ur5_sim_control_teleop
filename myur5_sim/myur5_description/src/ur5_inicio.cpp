#include <ros/ros.h>
#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_initial_joint_state_node");
    ros::NodeHandle nh;

    // Cargar el modelo URDF del robot UR5
    KDL::Tree ur5_tree;
    
    std::string urdf_path = ros::package::getPath("myur5_description");
	if(urdf_path.empty()) {
		ROS_ERROR("ur_description package path was not found");
	}
	urdf_path += "/urdf/myur5.urdf";

    if (!kdl_parser::treeFromFile(urdf_path, ur5_tree))
    {
        ROS_ERROR("Failed to parse URDF");
        return -1;
    }

    // Crear un objeto de cadena cinemática para el brazo del robot UR5
    KDL::Chain ur5_chain;
    ur5_tree.getChain("pedestal", "wrist_3_link", ur5_chain); // Ajusta los nombres de los eslabones según tu descripción URDF

    // Inicializar el solucionador de cinemática directa
    KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);

    // Inicializar el solucionador de matriz Jacobiana
    KDL::ChainJntToJacSolver jac_solver(ur5_chain);

    // Definir las posiciones articulares actuales
    KDL::JntArray q(ur5_chain.getNrOfJoints());
    for (size_t i = 0; i < q.rows(); ++i)
    {
        q(i) = 0.0; // Configura las posiciones articulares deseadas
    }

    // Calcular la matriz Jacobiana en la configuración actual
    KDL::Jacobian jac;
    jac_solver.JntToJac(q, jac);

    // Mostrar la matriz Jacobiana en la consola
    ROS_INFO("Matriz Jacobiana:");
    for (size_t i = 0; i < jac.rows(); ++i)
    {
        for (size_t j = 0; j < jac.columns(); ++j)
        {
            ROS_INFO("%f ", jac(i, j));
        }
        ROS_INFO("\n");
    }

    return 0;
}
