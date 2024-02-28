#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/frames.hpp>

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

int main()
{

  //Eigen and pinocchio variables
  Eigen::MatrixXd identity  = Eigen::Matrix<double, 6,6 >::Identity();
  Eigen::MatrixXd identity2  = Eigen::Matrix<double, 6,6 >::Identity();
  Eigen::VectorXd sum_error = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd last_error = Eigen::VectorXd::Zero(6);;
  Eigen::Matrix<double, 6, 6> matrix = Eigen::Matrix<double, 6, 6>::Ones();
  
  pinocchio::Data::Matrix6x J(6,7); // Pre - allocate the memory space. 
  J.setZero();  

  //Variables
  // double l = 0.0001;
  double l = 0.0001;
  const double damp = 1e-6;

  // Define the PID gains
  //  double Kp = 100.0;
  Eigen::VectorXd Kp(6);
  Kp << 10., 10., 10., 10., 10., 10.;
  double Ki = 0.1;
  // double Kd = 0.001;
  
  // Define robotDart model
  auto iiwa_robot = std::make_shared<robot_dart::robots::Iiwa>(); 
  iiwa_robot->set_actuator_types("servo");  
  
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

  //Test initial position data
  cout<<"Neutral pinocchio: "<<endl;
  // cout<<data.oMi[7].rotation()<<endl;
  // cout<<data.oMi[7].translation().transpose()<<endl;
  cout<<data.oMf[frame_id].rotation()<<endl;
  cout<<data.oMf[frame_id].translation().transpose()<<endl;

  // Set robot to target position.
  Eigen::VectorXd q_desired(model.nq);
  q_desired << 0.0, M_PI / 2, 0.0, M_PI / 2, 0.0, M_PI / 2, 0.0; // Set the desired joint angles here
  iiwa_robot->set_positions(q_desired);

  // Take target transformation matrix.
  auto eef_tf = iiwa_robot->body_pose("iiwa_link_ee");
  auto target_translation = eef_tf.translation();
  auto target_rotation = eef_tf.rotation();

  //Pinochio target SE3.
  //framesForwardKinematics(model,data,q);
  forwardKinematics(model,data,q_desired);
  pinocchio::updateFramePlacements(model, data);
  //pinocchio::SE3 pinocchio_target( data.oMi[7].rotation(),data.oMi[7].translation() );
  pinocchio::SE3 pinocchio_target( data.oMf[frame_id].rotation(),data.oMf[frame_id].translation() );

  cout<<"Target position pinocchio: "<<endl;
  cout<<pinocchio_target.rotation()<<endl;
  cout<<pinocchio_target.translation().transpose()<<endl;

  std::cout << iiwa_robot->body_pose("iiwa_link_ee").matrix() << std::endl;

  //Set robot at initial position.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  iiwa_robot->set_positions(q);
  //framesForwardKinematics(model,data,q);
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

  // End effector target twist
  Eigen::VectorXd end_effector_target_twist = log6(pinocchio_target).toVector(); 

  // Run the simulation loop
  while (true) {

      // Update pinocchio model joint configuration.
      Eigen::VectorXd joint_positions = iiwa_robot->positions();
      //framesForwardKinematics(model,data,q);
      forwardKinematics(model, data, joint_positions);
      updateFramePlacements(model, data);
      
      // Calculate error from target.
      Eigen::VectorXd err(6);
      // 1st way
      Eigen::Vector3d rotation_error = log3(pinocchio_target.rotation() * data.oMf[frame_id].rotation().transpose()); // log3( data.oMf[frame_id].rotation().transpose() * pinocchio_target.rotation()   );
      Eigen::Vector3d linear_error = pinocchio_target.translation() - data.oMf[frame_id].translation();
      
      err << linear_error, rotation_error;
      
      // 2nd way.
      //err = log6( data.oMf[frame_id].inverse() * pinocchio_target ).toVector();
      //err = log6( pinocchio_target.inverse() * data.oMf[frame_id] ).toVector();

      // std::cout << err.transpose() << std::endl;
      
      // Update control parameters.
      sum_error = sum_error + err * world_time_step;
      
      // Calculate velocity.
      
      Eigen::VectorXd velocity = Kp.array() * err.array() + Ki * sum_error.array();    
      // std::cout << velocity.transpose() << std::endl;
      
      // Compute Jacobian in world frame.
      J.setZero();
      
      // Get the Jacobian of the frame in the world frame 
      pinocchio::computeFrameJacobian(model, data, joint_positions, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
      //J = iiwa_robot->jacobian("iiwa_link_ee");

      // pinocchio::computeJointJacobians(model,data,q);
      // pinocchio::getJointJacobian(model,data,7,pinocchio::WORLD,J);
      
      // Compute damped pseudoinverse.
      Eigen::MatrixXd pseudoinverse = J.transpose() * (J * J.transpose() + l*l*identity2).inverse();
      Eigen::VectorXd cmd = pseudoinverse * velocity;
      
      //std::cout << "cmd: " << cmd.transpose() << std::endl;
      
      iiwa_robot->set_commands(cmd);
      //world_time_step = world_time_step + 0.
      if (simu.step_world(false, false)){
            break;
      }
  }
  iiwa_robot.reset();
  return 0;
}