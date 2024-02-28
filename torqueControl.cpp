#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robots/iiwa.hpp>
#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif
using namespace std;
int main()
{
    // Load the iiwa robot model in RobotDART
    
    auto iiwa_robot = std::make_shared<robot_dart::robots::Iiwa>(); 
    //Use torque actuators
    iiwa_robot->set_actuator_types("torque");
    // Define the PID gains
    double Kp = 100.0;
    double Ki = 0.01;
    double Kd = 0.1;
    
    // Define the desired joint positions 
    Eigen::VectorXd desired_positions = Eigen::VectorXd::Zero(iiwa_robot->positions().size());

    desired_positions[0] = 0;
    desired_positions[1] = M_PI / 2;
    desired_positions[2] = 0.5;
    desired_positions[3] = M_PI / 2;
    desired_positions[4] = 0;
    desired_positions[5] = M_PI / 2 ;
    desired_positions[6] = 0;

    //cout<<iiwa_robot->body_pose().transpose()<<endl;


    // Define the PID error variables
    Eigen::VectorXd error = Eigen::VectorXd::Zero(iiwa_robot->positions().size());
    Eigen::VectorXd last_error = Eigen::VectorXd::Zero(iiwa_robot->positions().size());
    Eigen::VectorXd integral_error = Eigen::VectorXd::Zero(iiwa_robot->positions().size());

    // Define the simulation time step
    double world_time_step = 0.001;
    robot_dart::RobotDARTSimu simu(0.001);
    #ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
    simu.set_graphics(graphics);

    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
    #endif
    
    simu.set_collision_detector("fcl");
    simu.add_checkerboard_floor();
    simu.add_robot(iiwa_robot);
    simu.set_text_panel("IIWA simulation");

    Eigen::Vector3d torque;
    Eigen::MatrixXd M;
    Eigen::VectorXd c;
    Eigen::VectorXd external_force2;
    Eigen::VectorXd external_force(7);
    
    // Run the simulation loop
    while (true) {            
        // Query the current joint positions and velocities of the iiwa robot
        Eigen::VectorXd positions = iiwa_robot->positions();
        Eigen::VectorXd velocities = iiwa_robot->velocities();

        // Calculate the PID error terms
        error = desired_positions - positions;
        integral_error += error * world_time_step;
        Eigen::VectorXd derivative_error = (error - last_error) / world_time_step;

        // Calculate the required joint velocities using the velocity PID controller and the desired joint positions and velocities
        Eigen::VectorXd vel = Kp * error + Ki * integral_error + Kd * derivative_error;
        
        M = iiwa_robot->mass_matrix();
        c = iiwa_robot->coriolis_gravity_forces();
        //
        external_force = M*( Kp * error + Ki * integral_error + Kd * derivative_error ) + c;
        //Simple pid
        //external_force = Kp * error + Ki * integral_error + Kd * derivative_error ;
        
        //cout<<"1: "<<external_force.transpose()<<endl;
        //cout<<iiwa_robot->positions().transpose()<<endl;
        // Send the calculated joint torques to the iiwa robot
        iiwa_robot->set_commands(external_force);
        //cout<<"2: "<<iiwa_robot->forces()<<endl; 
        
        //Velocity control.
        //iiwa_robot->set_velocities(vel);

        //Torque control.
        // Calculate the required joint torques using the torque PID controller and the desired joint positions and velocities
        //torque = Kp * error + Ki * integral_error + Kd * derivative_error;
        //M = iiwa_robot->mass_matrix();
        //c = iiwa_robot->coriolis_gravity_forces();

        // Update the last error
        last_error = error;
        
        if (simu.step_world()){
            break;
        }
    }
    
    iiwa_robot.reset();
    return 0;

}