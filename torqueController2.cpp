#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robots/iiwa.hpp>

#include <iostream>
#include <string>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif
using namespace std;
using namespace pinocchio;
int main(){

    Eigen::VectorXd error_in_world_frame = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd sum_error = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd identity2  = Eigen::Matrix<double, 6,6 >::Identity();

    pinocchio::Data::Matrix6x J(6,7); // Pre - allocate the memory space. 
    J.setZero();  

    // Define robotDart model
    auto iiwa_robot = std::make_shared<robot_dart::robots::Iiwa>(); 
    //auto iiwa_robot = std::make_shared<robot_dart::Robot>("/home/baknis/Documents/dartProject/iiwa.urdf");
    iiwa_robot->set_actuator_types("torque");  
    
    //Load ghost robot for visualization.
    auto robot_ghost = iiwa_robot->clone_ghost();

    //Pinocchio model urdf
    string urdfPath = "/home/baknis/Documents/dartProject/iiwa.urdf";

    // Build pinocchio model and data.
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdfPath, model);
    pinocchio::Data data(model);
    Eigen::VectorXd q = pinocchio::neutral(model);

    //Get Frame id of end effector
    int frame_id = model.getFrameId("iiwa_link_ee");
    cout<<frame_id<<endl;

    // Set robot to target position.
    Eigen::VectorXd q_desired(7);
    q_desired << 0.0, M_PI / 2, 0.0, M_PI / 2, 0.0, M_PI / 2, 0.0; // Set the desired joint angles here
    iiwa_robot->set_positions(q_desired);

    // Take target transformation matrix.
    auto eef_tf = iiwa_robot->body_pose("iiwa_link_ee");
    auto target_translation = eef_tf.translation();
    auto target_rotation = eef_tf.rotation();
    
    //Pinochio target SE3.
    forwardKinematics(model,data,q_desired);
    updateFramePlacements(model, data);
    SE3 pinocchio_target( data.oMf[frame_id].rotation(),data.oMf[frame_id].translation() );

    std::cout << iiwa_robot->body_pose("iiwa_link_ee").matrix() << std::endl;

    //Set robot at initial position.
    q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    iiwa_robot->set_positions(q);

    forwardKinematics(model,data,q);
    pinocchio::updateFramePlacements(model, data);

    double world_time_step = 0.001;
    robot_dart::RobotDARTSimu simu(world_time_step);
    #ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
    simu.set_graphics(graphics);

    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
    #endif

    simu.set_collision_detector("fcl");
    simu.add_checkerboard_floor();
    simu.add_robot(iiwa_robot);
    robot_ghost->set_positions(q_desired);
    simu.add_robot(robot_ghost);
    simu.set_text_panel("IIWA simulation");

    //Variables
    double l = 0.0001;

    // Define the PID gains
    auto Kp = 100.0;
    auto Ki = 0.001;
    // double Kd = 0.001;
    // Run the simulation loop
    auto dofs = iiwa_robot->dof_names();
    while (true) {

        if (simu.step_world(false, false)){
            break;
        }
        
        //auto eef_tf = iiwa_robot->body_pose("iiwa_link_ee");
        
        auto joint_positions = iiwa_robot->positions();

        framesForwardKinematics(model,data,joint_positions);

        auto rotation_error = log3(pinocchio_target.rotation() * data.oMf[frame_id].rotation().transpose() );

        auto linear_error = pinocchio_target.translation() - data.oMf[frame_id].translation();

        error_in_world_frame << linear_error , rotation_error;



        sum_error = sum_error + error_in_world_frame * world_time_step ; 

        Eigen::VectorXd velocity = Kp*error_in_world_frame + Ki*sum_error;
        
       
        
        J.setZero();  
        computeFrameJacobian(model, data, joint_positions, frame_id, LOCAL_WORLD_ALIGNED , J);
        auto jac_pinv = J.transpose() * (J * J.transpose() + l*l*identity2).inverse();
        //auto g = iiwa_robot->gravity_forces(dofs);

        //computeGeneralizedGravity(model, data, joint_positions);

        // auto cmd = J.transpose() * velocity + g;
        
        auto h = iiwa_robot->coriolis_gravity_forces(dofs);
        
        //auto m = iiwa_robot->mass_matrix(dofs);
        crba(model,data,joint_positions);
        auto cmd = data.M * ( jac_pinv * velocity) + h ;
        
        iiwa_robot->set_commands(cmd);

        

        // iiwa_robot->set_commands(cmd);

        
    }
}