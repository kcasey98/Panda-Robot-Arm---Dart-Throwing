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
 
#define RAD(deg) ((double)(deg) * M_PI / 180.0)
 
#include "redis_keys.h"
 
// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";
 
string xpos;
string ypos;
string zpos;

enum State
{
   POSTURE = 0,
   MOTION = 1,
   LAUNCH1 = 2,
   PREGRAB = 3,
   BEGINNING = 4,
   GRAB = 5,
   POSTGRAB = 6,
   PRELAUNCH = 7,
   ORIENTATE = 8,
   TRAJ2= 9,
   PRETRAJ2 = 10,
   LINEARTRAJ = 11,
};
 
int main() {
 
   // initial state
   int state = PREGRAB;
   // MOTION or PREGRAB
   string controller_status = "1";
  
   // start redis client
   auto redis_client = RedisClient();
   redis_client.connect();
   redis_client.set("target_board", "1");  // communicate target to simviz - remove once GUI is incorporated
 
   Vector3d X_plan, x, dx, X_der, X_der2;
 
   MatrixXd X_traj(10,1);
 
   double dt = 0.001;
 
   // set up signal handler
   signal(SIGABRT, &sighandler);
   signal(SIGTERM, &sighandler);
   signal(SIGINT, &sighandler);
 
   // read trajectory **************** William
   // const string pos_traj_fname = "pos_traj.txt";
   // const string vel_traj_fname = "vel_traj.txt";
   // auto pos_traj = readMatrix(pos_traj_fname.c_str());
   // auto vel_traj = readMatrix(vel_traj_fname.c_str());
 
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

    double angle_throw = 0;
    double angle_final;

   initial_pos << x(0), x(1), x(2);

   VectorXd posori_task_torques = VectorXd::Zero(dof);
   posori_task->_kp_pos = 6000.0;
   posori_task->_kv_pos = 155.0;
   posori_task->_kp_ori = 800.0;
   posori_task->_kv_ori = 50.0;
 
   // joint task
   auto joint_task = new Sai2Primitives::JointTask(robot);
   joint_task->_use_interpolation_flag = true;
   joint_task->_use_velocity_saturation_flag = true;
 
   VectorXd joint_task_torques = VectorXd::Zero(dof);
   joint_task->_kp = 200.0;
   joint_task->_kv = 40.0;
 
   VectorXd q_init_desired(dof);
   VectorXd init_desired(dof);
   VectorXd q_zero(dof);
   q_zero << 0,0,0,0,0,0,0;
   init_desired = robot->_q;
   q_init_desired << 0, -10, 0, -100.0, 0.0, 180, 45.0;
   q_init_desired *= M_PI/180.0;
   

   // q_init_desired << 0, -0.6, 0, 0.8, 0, M_PI, M_PI_4;
   //q_init_desired << 0, 0.2, 0, -1.5, 0, M_PI, M_PI_4;
   
//    q_init_desired << 0, -0.6, 0, 0.8, 0, M_PI, M_PI_4;
   joint_task->_desired_position = q_init_desired;
 
   // gripper task containers
   VectorXd gripper_command_torques(2);
   VectorXd q_gripper(2), dq_gripper(2);
   VectorXd q_gripper_desired(2);
   //q_gripper_desired.setZero();
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
   //LoopTimer timer2;
   //double start_time2;
 
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
 
       if (state == PREGRAB) { //part 1
           
           redis_client.set("my_test_key", "Running Controller");
           
           // update task model and set hierarchy
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
 
           if (x(0) > 0.52 && x(2) < 0.84 && dx(2) < 0.001){
                state = GRAB;
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                robot->rotation(R_init,control_link);
                // q_gripper_desired << -0.02, 0.02;
           }
       } else if (state == POSTURE) { //part 8
           // update task model and set hierarchy
           q_init_desired << 0, -10, 0, -100.0, 0.0, 180, 45.0;
            q_init_desired *= M_PI/180.0;
            joint_task->_desired_position = q_init_desired;
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

       } else if (state == GRAB){ //part2
           //call the following 4 lines every time you switch tasks
           N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
           //joint_task->_desired_position = q_init_desired;
 
        //    posori_task->_desired_position = Vector3d(0.6, 0, 0.73); //gets us to dart
            posori_task->_desired_position = Vector3d(0.6, 0, 0.69); //gets us to dart
           posori_task->_desired_orientation = R_init* AngleAxisd(0, Vector3d(0, 1, 0)).toRotationMatrix(); //perpendicular to dart
           
            // now grab
            //Mikael said to create new task for grabbing
           if ((x - Vector3d(0.6, 0, 0.69)).norm() <= 0.0001){
               q_gripper_desired << -0.019, 0.019;
               //cout << (q_gripper-q_gripper_desired).norm() << endl;
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
          
       } else if (state == POSTGRAB){ //part 3
           //is postgrab needed? unsure
           N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
           //joint_task->_desired_position = q_init_desired;
 
           posori_task->_desired_position = Vector3d(0.3, 0, 1);
        //    q_init_desired << -M_PI_2, M_PI_2, 0, 0, 0, M_PI, M_PI_4;
        //    joint_task->_desired_position = q_init_desired;
           // posori_task->_desired_orientation = R_init * AngleAxisd(angle, Vector3d(0, 1, 0)).toRotationMatrix();  //william
 
           // posori_task->_desired_orientation = AngleAxisd(angle, Vector3d(0, 1, 0)).toRotationMatrix();
           posori_task->computeTorques(posori_task_torques);
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = posori_task_torques + joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
           if (x(2) > 0.99){
            //    cout << robot->_q << endl;
               //state = PRELAUNCH;
               state = ORIENTATE;
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               init_desired = robot->_q; //save joint orientation
           }
       } else if (state == PRELAUNCH){ //part 5
           //moves robot into prelaunch position right before trajectory
            // don't know if this is optimal, have been playing around based pn TA consulting

            q_init_desired << 90, -90, 0, -90, 0, 180, 45; //works well 6 am
            // q_init_desired << 90, -90, 0, 0, 0, 180, 45;
           q_init_desired *= (M_PI/180);
           joint_task->_desired_position = q_init_desired;
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

           if ((q_init_desired-robot->_q).norm() < 0.01){
            //    cout << "hello" << endl;
            //    cout << x << endl;
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               state = PRETRAJ2;

                // //mikael added today 5/24
                // posori_task->_use_velocity_saturation_flag = true; //this does nothing
                // posori_task->_linear_saturation_velocity = 1000.0;
                // posori_task->_angular_saturation_velocity = M_PI_2;

               robot->rotation(R_init,control_link);

                angle_throw = acos(R_init.col(2).dot(Vector3d::UnitZ())); //current angle with x axis of world
           }
       } else if (state == ORIENTATE){ //part 4
           q_init_desired = init_desired;
           q_init_desired(0) = M_PI_2;
           
           joint_task->_desired_position = q_init_desired;
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

        //    cout << (q_init_desired-robot->_q).norm() << endl;
           
           if ((q_init_desired-robot->_q).norm() < 0.006){
               joint_task->_saturation_velocity = (M_PI/10)*VectorXd::Ones(9); //slow down joint task velocity to keep dart still/perpendicular
               state = PRELAUNCH;
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               robot->rotation(R_init,control_link);
           }
       } else if(state == PRETRAJ2){ //part 6
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
       } else if (state == LINEARTRAJ){ //part 7

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


            //orientation

            // angle_final = atan2(2,2);
            //angle_final = atan2(1,sqrt(3));

            // if ((x-Vector3d(0, 0, 1.1)).norm() < 1){
            // if (x(1) > -0.3){
            //posori_task->_desired_orientation = AngleAxisd(angle_final, Vector3d(1, 0, 0)).toRotationMatrix()*AngleAxisd(-M_PI_4, Vector3d(0, 0, 1)).toRotationMatrix();
            // }

            // if ((x-Vector3d(0, -0.15, 0.95)).norm() < 0.01){
            // MOMENT WHERE GRIPPER WILL RELEASE / THROW DART
            cout << "time2" << endl;
            cout << time2 << endl;
            cout << "yzplan" << endl;
            cout << -0.693 + 0.5*ay*time2*time2 << endl;
            cout << 0.7 + 0.5*az*time2*time2 << endl;
            // cout << -0.5 + 0.75*9.81*time2*time2 << endl;
            // cout << 0.6 + 0.75*9.81*time2*time2 << endl;
            cout << "dyzplan" << endl;
            cout << ay*time2 << endl;
            cout << az*time2 << endl;
            // cout << 1.5*9.81*time2 << endl;
            // cout << 1.5*9.81*time2 << endl;
            cout << "x" << endl;
            cout << x << endl;
            cout << "dx" << endl;
            cout << dx << endl;

            // if (dx(1) >= 3.6){
            // if (time2>=0.297119){
            if (time2>=t_release){
                
                q_gripper_desired << -0.1,0.1;
                
                cout << "Sending a message to the redis client !!\n" << endl;
                redis_client.set("my_test_key", "Released !!!");
            }

            xpos = to_string(x(0));

            redis_client.set("my_test_key", xpos);

            posori_task->computeTorques(posori_task_torques);
               joint_task->computeTorques(joint_task_torques);
               command_torques.head(7) = posori_task_torques + joint_task_torques;
 
               gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
               command_torques.tail(2) = gripper_command_torques;

            // if (dx(1)>=3.6){
            if (time2>=t_release){
            // if (time2 >= 0.2487){
            // if (time2>=0.297119){
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                //joint_task->_desired_position << -M_PI_2, 0, 0, M_PI_2, 0, M_PI, M_PI_4;
                joint_task->_kp = 10;
                joint_task->_kv = 400.0;
                state = POSTURE;
            }

            if(counter2 % 10 == 0)
            {
                data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
                data_file << 0 << '\t' << -0.693 + 0.5*ay*time2*time2 << '\t' << 0.7 + 0.5*az*time2*time2 << '\t';
                data_file << dx(0) << '\t' << dx(1) << '\t' << dx(2) << '\t';
                data_file << 0 << '\t' << ay*time2 << '\t' << az*time2 << '\n';
            }
            // if(counter2 % 10 == 0)
            // {
            //     data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
            //     data_file << 0 << '\t' << -0.5 + 0.75*9.81*time2*time2 << '\t' << 0.6 + 0.75*9.81*time2*time2 << '\t';
            //     data_file << dx(0) << '\t' << dx(1) << '\t' << dx(2) << '\t';
            //     data_file << 0 << '\t' << 1.5*9.81*time2 << '\t' << 1.5*9.81*time2<< '\n';
            // }
            counter2++;
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
