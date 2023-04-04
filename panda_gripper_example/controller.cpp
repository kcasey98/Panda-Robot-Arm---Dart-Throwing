/**
* @file controller.cpp
* @brief Controller file
*
*/
 
#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
// #include "readEigen.h"
 
#include <iostream>
#include <string>
 
using namespace std;
using namespace Eigen;
 
#include <fstream>
#include <Eigen/Dense>
 
#define MAXBUFSIZE  ((int) 1e5)
 
#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

void update_torques(Sai2Primitives::JointTask *& joint_task, MatrixXd N_prec, VectorXd joint_task_torques,VectorXd& command_torques, VectorXd q_gripper, VectorXd dq_gripper, VectorXd& gripper_command_torques, VectorXd q_gripper_desired, double kv_gripper, double kp_gripper);
 
#define RAD(deg) ((double)(deg) * M_PI / 180.0)
 
#include "redis_keys.h"
 
// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";
 
string xpos;
string ypos;
string zpos;

enum State
{
   PREGRAB = 1,
   GRAB = 2,
   POSTGRAB = 3,
   ORIENTATE = 4,
   PRELAUNCH = 5,
   PRETRAJ2 = 6,
   LINEARTRAJ = 7,
   POSTURE = 8,
};
 
int main() {
 
   // initial state
   int state = PREGRAB;
   // Controller Status
   string controller_status = "1";
  
   // start redis client
   auto redis_client = RedisClient();
   redis_client.connect();
   redis_client.set("target_board", "4");  // communicate target to simviz - remove once GUI is incorporated
 
   Vector3d X_plan, x, dx, X_der, X_der2;
 
   MatrixXd X_traj(10,1);
 
   double dt = 0.001;
 
   // set up signal handler
   signal(SIGABRT, &sighandler);
   signal(SIGTERM, &sighandler);
   signal(SIGINT, &sighandler);
 
   // load robots, read current state and update the model
   auto robot = new Sai2Model::Sai2Model(robot_file, false);
   robot->_q.head(7) = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
   robot->_dq.head(7) = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
   robot->updateModel();
 
   // prepare controller
   int dof = robot->dof();
   VectorXd command_torques = VectorXd::Zero(dof + 2);  // panda + gripper torques
   MatrixXd N_prec = MatrixXd::Identity(dof, dof);
 
   // pose task
   const string control_link = "link7";
   const Vector3d control_point = Vector3d(0, 0, 0.22);
   auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
   posori_task->_use_interpolation_flag = true;
   posori_task->_use_velocity_saturation_flag = true;

   posori_task->setDynamicDecouplingFull(); //Mikael added this in OH, dynamic decoupling
   
   const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.22);
   // containers
   Vector3d ee_pos;
   Matrix3d ee_rot;
   Matrix3d R_init;
   
   //OTG
   robot->position(x, "link7", pos_in_ee_link);
   //robot->rotation(R_init,control_link);
   VectorXd initial_pos = VectorXd::Zero(3);

   initial_pos << x(0), x(1), x(2);

    // Set position torque variables
   VectorXd posori_task_torques = VectorXd::Zero(dof);
   posori_task->_kp_pos = 6000.0;
   posori_task->_kv_pos = 155.0;
   posori_task->_kp_ori = 800.0;
   posori_task->_kv_ori = 50.0;
 
   // joint task
   auto joint_task = new Sai2Primitives::JointTask(robot);
   joint_task->_use_interpolation_flag = true;
   joint_task->_use_velocity_saturation_flag = true;
 
    // Set joint torques variables
   VectorXd joint_task_torques = VectorXd::Zero(dof);
   joint_task->_kp = 200.0;
   joint_task->_kv = 40.0;
 
    // Set Joint containers
   VectorXd q_init_desired(dof);
   VectorXd init_desired(dof);
   VectorXd q_zero(dof);
   q_zero << 0,0,0,0,0,0,0;
   init_desired = robot->_q;
   q_init_desired << 0, -10, 0, -100.0, 0.0, 180, 45.0;
   q_init_desired *= M_PI/180.0;
   
   // Set initial desired joint space
   joint_task->_desired_position = q_init_desired;
 
   // Gripper task containers
   VectorXd gripper_command_torques(2);
   VectorXd q_gripper(2), dq_gripper(2);
   VectorXd q_gripper_desired(2);
   q_gripper_desired << -0.1, 0.1;
   double kp_gripper = 700;
   double kv_gripper = 40;
 
    VectorXd coriolis = VectorXd::Zero(dof);

   // setup redis callback
   redis_client.createReadCallback(0);
   redis_client.createWriteCallback(0);
 
   // add to read callback
   redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
   redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
   redis_client.addEigenToReadCallback(0, GRIPPER_JOINT_ANGLES_KEY, q_gripper);
   redis_client.addEigenToReadCallback(0, GRIPPER_JOINT_VELOCITIES_KEY, dq_gripper);
 
   // add to write callback
   redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
   redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
 
   // create a timer
   LoopTimer timer;
   timer.initializeTimer();
   timer.setLoopFrequency(1000);
   double start_time = timer.elapsedTime(); //secs
   bool fTimerDidSleep = true;
 
    // Set counters
   unsigned long long counter = 0;
   unsigned long long counter2 = 0;
 
   runloop = true;
    int controller_number = 1;

   string filename;
    if(controller_number == 1)
        filename = "../../panda_gripper_example/data_files/question_1.txt";

    ofstream data_file;
    data_file.open(filename);

   while (runloop) {

       // wait for next scheduled loop
       timer.waitForNextLoop();
       double time = timer.elapsedTime() - start_time;
 
       // execute redis read callback
       redis_client.executeReadCallback(0);
 
       robot->position(x, "link7", pos_in_ee_link);
       robot->linearVelocity(dx, "link7", pos_in_ee_link);
 
       // update model
       robot->updateModel();
       robot->coriolisForce(coriolis);


        // Enter Different States

       if (state == PREGRAB) { //part 1: The robot moves into the correct joint orientation
        redis_client.set("my_test_key", "Running Controller");
        
        // update task model and set hierarchy and compute torques
        update_torques(joint_task, N_prec, joint_task_torques, command_torques, q_gripper, dq_gripper, gripper_command_torques, q_gripper_desired, kv_gripper, kp_gripper);

        // if 
        if (x(0) > 0.52 && x(2) < 0.84 && dx(2) < 0.001){
            state = GRAB;
            joint_task->reInitializeTask();
            posori_task->reInitializeTask();
            robot->rotation(R_init,control_link);
            // q_gripper_desired << -0.02, 0.02;
        }

       } else if (state == GRAB){ //part2: The robot now adjusts the end-effector

           //call the following 4 lines every time you switch tasks
           N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
 
           posori_task->_desired_position = Vector3d(0.6, 0, 0.69); //gets us to dart
           posori_task->_desired_orientation = R_init* AngleAxisd(0, Vector3d(0, 1, 0)).toRotationMatrix(); //perpendicular to dart
           
            // if robot is close, it must now grab the dart
           if ((x - Vector3d(0.6, 0, 0.69)).norm() <= 0.0001){
               q_gripper_desired << -0.019, 0.019;
               // if the the grippers have the dart
               if ((q_gripper - q_gripper_desired).norm() < 0.004){
                   state = POSTGRAB;
                   joint_task->reInitializeTask();
                   posori_task->reInitializeTask();
               }
           }

           posori_task->computeTorques(posori_task_torques);
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = posori_task_torques + joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
          
       } else if (state == POSTGRAB){ //part 3: dart is grabbed and now we must separate from the table
           //call the following 4 lines every time you switch tasks
           N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
 
           posori_task->_desired_position = Vector3d(0.3, 0, 1);
 
           posori_task->computeTorques(posori_task_torques);
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = posori_task_torques + joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

           // if the robot is close to desired position
           if (x(2) > 0.99){
               state = ORIENTATE;
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               init_desired = robot->_q; //save joint orientation
           }
       } else if (state == ORIENTATE){ //part 4: use joint space to rotate base so dart doesn't fall
            cout << state << endl;
           q_init_desired = init_desired;
           q_init_desired(0) = M_PI_2;
           
           joint_task->_desired_position = q_init_desired;

           // update task model and set hierarchy and compute torques
           update_torques(joint_task, N_prec, joint_task_torques, command_torques, q_gripper, dq_gripper, gripper_command_torques, q_gripper_desired, kv_gripper, kp_gripper);
           
           if ((q_init_desired-robot->_q).norm() < 0.006){
               joint_task->_saturation_velocity = (M_PI/10)*VectorXd::Ones(9); //slow down joint task velocity to keep dart still/perpendicular
               state = PRELAUNCH;
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               robot->rotation(R_init,control_link);
           }
       } else if (state == PRELAUNCH){ //part 5: moves robot into prelaunch position right before trajectory
            cout << state << endl;
           q_init_desired << 90, -90, 0, -90, 0, 180, 45;
           q_init_desired *= (M_PI/180); // convert to radians
           joint_task->_desired_position = q_init_desired;

           // update task model and set hierarchy and compute torques
           update_torques(joint_task, N_prec, joint_task_torques, command_torques, q_gripper, dq_gripper, gripper_command_torques, q_gripper_desired, kv_gripper, kp_gripper);

           if ((q_init_desired-robot->_q).norm() < 0.01){
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               state = PRETRAJ2;
               robot->rotation(R_init,control_link);
           }
       } else if(state == PRETRAJ2){ //part 6: before trajectory, move robot into correct position space
           N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
           
            posori_task->_desired_position = Vector3d(0, -0.693, 0.7); //gets us to dart
           //posori_task->_desired_orientation = R_init* AngleAxisd(0, Vector3d(0, 1, 0)).toRotationMatrix(); //perpendicular to dart

           if ((x - Vector3d(0, -0.693, 0.7)).norm() <= 0.002){
                state = LINEARTRAJ;
                // state = POSTURE;
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
           }

           posori_task->computeTorques(posori_task_torques);
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = posori_task_torques + joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

       } else if (state == LINEARTRAJ){ //part 7: Trajectory

            // Create trajectory variables
            double time2 = counter2*dt;
            double delta_y = 4;
            double dz = 0.4;
            double dy = dz*sqrt(3);
            double vz = sqrt((delta_y*9.81)/(2*sqrt(3)));
            double vy = sqrt(3)*vz;
            double az = (vz*vz)/(2*dz);
            double ay = (vy*vy)/(2*dy);
 
            double t_release = vz/az;
            cout << "tr" << endl;
            cout << t_release << endl;


           posori_task->_ki_pos = 500.0;

           posori_task->_use_interpolation_flag = false;
           posori_task->_use_velocity_saturation_flag = true;
           posori_task->_linear_saturation_velocity = 500;

            N_prec.setIdentity();
            posori_task->updateTaskModel(N_prec);
            N_prec = posori_task->_N;
            joint_task->updateTaskModel(N_prec);


            //  // dartboard1 30 60
            // posori_task->_desired_position = Vector3d(0, -0.5 + 0.25*9.81*time2*time2, 0.6 + 0.25*9.81*time2*time2);
            // posori_task->_desired_velocity = Vector3d(0, 0.5*9.81*time2, 0.5*9.81*time2);
            // posori_task->_desired_acceleration = Vector3d(0, 9.81*0.5, 9.81*0.5);

            // // dartboard2 30 60
            // posori_task->_desired_position = Vector3d(0, -0.5 + 0.5*9.81*time2*time2, 0.6 + 0.5*9.81*time2*time2);
            // posori_task->_desired_velocity = Vector3d(0, 9.81*time2, 9.81*time2);
            // posori_task->_desired_acceleration = Vector3d(0, 9.81, 9.81);

            // // dartboard3 30 60 
            // posori_task->_desired_position = Vector3d(0, -0.693 + 0.5*ay*time2*time2 + 0.5*time2, 0.2*time2 + 0.7 + 0.5*az*time2*time2);
            // posori_task->_desired_velocity = Vector3d(0, 1.1 + ay*time2, -0.5 + az*time2);
            // posori_task->_desired_acceleration = Vector3d(0, ay, az);

            // dartboard4 30 60 
            posori_task->_desired_position = Vector3d(0, -0.693 + 0.5*ay*time2*time2 + 0.5*time2, 0.2*time2 + 0.7 + 0.5*az*time2*time2);
            posori_task->_desired_velocity = Vector3d(0, 1.2 + ay*time2, -0.4 + az*time2);
            posori_task->_desired_acceleration = Vector3d(0, ay, az);

            // MOMENT WHERE GRIPPER WILL RELEASE / THROW DART
            if (time2>=t_release){
                
                q_gripper_desired << -0.1,0.1;
                
                cout << "Sending a message to the redis client !!\n" << endl;
                redis_client.set("my_test_key", "Released !!!");
            }

            // set the test key to the robots xpos for simviz trajectory
            xpos = to_string(x(0));
            redis_client.set("my_test_key", xpos);

            // set torques
            posori_task->computeTorques(posori_task_torques);
            joint_task->computeTorques(joint_task_torques);
            command_torques.head(7) = posori_task_torques + joint_task_torques;

            gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
            command_torques.tail(2) = gripper_command_torques;

            // dart is released, now relax
            if (time2>=t_release){
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                joint_task->_kp = 10;
                joint_task->_kv = 400.0;
                state = POSTURE;
            }

            // write to data file for graph
            if(counter2 % 10 == 0)
            {
                data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
                data_file << 0 << '\t' << -0.693 + 0.5*ay*time2*time2 << '\t' << 0.7 + 0.5*az*time2*time2 << '\t';
                data_file << dx(0) << '\t' << dx(1) << '\t' << dx(2) << '\t';
                data_file << 0 << '\t' << ay*time2 << '\t' << az*time2 << '\n';
            }
            counter2++;

       } else if (state == POSTURE) { //part 8: move to final joint config after throw
           // Set final joint position
           q_init_desired << 0, -10, 0, -100.0, 0.0, 180, 45.0;
            q_init_desired *= M_PI/180.0;
            joint_task->_desired_position = q_init_desired;

            // update task model and set hierarchy and compute torques
           update_torques(joint_task, N_prec, joint_task_torques, command_torques, q_gripper, dq_gripper, gripper_command_torques, q_gripper_desired, kv_gripper, kp_gripper);

       }
       // execute redis write callback
       redis_client.executeWriteCallback(0);
 
       counter++;
   }

   data_file.close();
 
   double end_time = timer.elapsedTime();
   std::cout << "\n";
   std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
   std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
   std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
 
   redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating
 
   return 0;
}


void update_torques(Sai2Primitives::JointTask *& joint_task, MatrixXd N_prec, VectorXd joint_task_torques,VectorXd& command_torques, VectorXd q_gripper, VectorXd dq_gripper, VectorXd& gripper_command_torques, VectorXd q_gripper_desired, double kv_gripper, double kp_gripper){
    // update task model and set hierarchy
    N_prec.setIdentity();
    joint_task->updateTaskModel(N_prec);

    // compute torques
    joint_task->computeTorques(joint_task_torques);
    command_torques.head(7) = joint_task_torques;

    gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
    command_torques.tail(2) = gripper_command_torques;

}
