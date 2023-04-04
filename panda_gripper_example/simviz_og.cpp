/**
 * @file simviz.cpp
 * @brief Simulation + visualization
 *
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"
#include "../include/object.h"

using namespace std;
using namespace Eigen;

// specify urdf and robots
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm_hand.urdf";
const string robot_name = "panda_arm_hand";
const string camera_name = "camera_fixed";
const string base_link_name = "link0";
const string ee_link_name = "link7";

// dynamic objects information
const vector<string> object_names = {"dart"};
vector<Vector3d> object_pos;
vector<Vector3d> object_lin_vel;
Vector3d dart_hit;  // Final position vector for the dart once it hits the board
VectorXd q_launch(9);
vector<Quaterniond> object_ori;
vector<Vector3d> object_ang_vel;
const int n_objects = object_names.size();
double vy;
double vz;
double time2 = 0;

// redis client
RedisClient redis_client;

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;
bool bd = false;
bool hit = false;
bool lol = false;
bool release = false;
bool traj = false;
auto TEST_KEY_STRING = "my_test_key";  // want to receive from controller
auto BOARD_TARGET = "target_board";  // want to receive from controller


int main() {
    
    cout << "Loading URDF world model file: " << world_file << endl;

    // start redis client
    redis_client = RedisClient();
    redis_client.connect();
	// redis_client.set("target_board", "0"); // initialized so that no target is selected

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
    Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    graphics->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background
    // graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
    graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes

    //redis calls
    double dartboard_distance; // = redis...
    //graphics->updateObjectGraphics("dartboard", Vector3d(0,4,1), Quaterniond(sqrt(2)/2,sqrt(2)/2, 0, 0));

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    // robot->_q = VectorXd::Zero(dof);
    VectorXd q_init_desired(9);
    // q_init_desired << -M_PI_2, M_PI_2, 0, M_PI_2, 0, M_PI, M_PI_4;
    q_init_desired << -M_PI_2, 0, 0, M_PI_2, 0, M_PI, M_PI_4, 0, 0;
    robot->_q = q_init_desired;
    // robot->_dq = VectorXd::Zero(dof);
    // robot->updateKinematics();
    q_launch << 1.57291,0.848205,0.002892,0.950951,-0.005601,3.70764,0.787863,-0.016721,0.024347;


    robot->updateModel();


    // load simulation world
    auto sim = new Simulation::Sai2Simulation(world_file, false);
    sim->setJointPositions(robot_name, robot->_q);
    sim->setJointVelocities(robot_name, robot->_dq);

    // fill in object information
    for (int i = 0; i < n_objects; ++i) {
        Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
        Quaterniond _object_ori;
        sim->getObjectPosition(object_names[i], _object_pos, _object_ori);
        sim->getObjectVelocity(object_names[i], _object_lin_vel, _object_ang_vel);
        object_pos.push_back(_object_pos);
        object_lin_vel.push_back(_object_lin_vel);
        object_ori.push_back(_object_ori);
        object_ang_vel.push_back(_object_ang_vel);
    }

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.01);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(10);
    sim->setCoeffFrictionDynamic(10);

    /*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
    int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Panda Starter Code", NULL, NULL);
    glfwSetWindowPos(window, windowPosX, windowPosY);
    glfwShowWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetKeyCallback(window, keySelect);
    glfwSetMouseButtonCallback(window, mouseClick);

    // cache variables
    double last_cursorx, last_cursory;

    // init redis client values
    redis_client.set(CONTROLLER_RUNNING_KEY, "0");
    redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
    redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);

    // start simulation thread
    thread sim_thread(simulation, robot, sim);

    // initialize glew
    glewInitialize();

    // while window is open:
    int count = 0;
    Vector3d start_pos = Vector3d(1, -1, 1);

	auto target = redis_client.get(BOARD_TARGET);

    while (!glfwWindowShouldClose(window) && fSimulationRunning)
    {
        // // add sphere for every nth count
        // if (count % 60 == 0) {  // default refresh rate
        //     addSphere(graphics, "test", start_pos, Quaterniond(1, 0, 0, 0), 0.01, Vector4d(1, 1, 1, 1));
        //     addBox(graphics, "test", start_pos + Vector3d(-2, 0, 0), Quaterniond(1, 0, 0, 0), Vector3d(0.05, 0.05, 0.05), Vector4d(1, 1, 1, 1));
        //     start_pos(1) += 1e-1;
        // }

        // update graphics. this automatically waits for the correct amount of time
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot_name, robot);
        for (int i = 0; i < n_objects; ++i) {
            graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
        }
        // only one dartboard
        //graphics->updateObjectGraphics("dartboard", Vector3d(0,4,-10), Quaterniond(0.5, 0.5, -0.5, 0.5));
        //<origin xyz="0 4 1" rpy="1.5707 0.0 0" />
		if(target == "4"){
			graphics->updateObjectGraphics("dartboard4", Vector3d(0,4,1.1), Quaterniond(0.5, 0.5, -0.5, 0.5));
		}
		else if(target == "3"){
			graphics->updateObjectGraphics("dartboard3", Vector3d(0,3,1.1), Quaterniond(0.5, 0.5, -0.5, 0.5));
		}
		else if(target == "2"){
			graphics->updateObjectGraphics("dartboard2", Vector3d(0,2,1.1), Quaterniond(0.5, 0.5, -0.5, 0.5));
		}
		else if(target == "1"){
			graphics->updateObjectGraphics("dartboard", Vector3d(0,1,1.1), Quaterniond(0.5, 0.5, -0.5, 0.5));
		}
        graphics->render(camera_name, width, height);

        // swap buffers
        glfwSwapBuffers(window);

        // wait until all GL commands are completed
        glFinish();

        // check for any OpenGL errors
        GLenum err;
        err = glGetError();
        assert(err == GL_NO_ERROR);

        // poll for events
        glfwPollEvents();

        // move scene camera as required
        // graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
        Eigen::Vector3d cam_depth_axis;
        cam_depth_axis = camera_lookat - camera_pos;
        cam_depth_axis.normalize();
        Eigen::Vector3d cam_up_axis;
        // cam_up_axis = camera_vertical;
        // cam_up_axis.normalize();
        cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
        Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
        cam_roll_axis.normalize();
        Eigen::Vector3d cam_lookat_axis = camera_lookat;
        cam_lookat_axis.normalize();

        if (fTransXp) {
            camera_pos = camera_pos + 0.05*cam_roll_axis;
            camera_lookat = camera_lookat + 0.05*cam_roll_axis;
        }
        if (fTransXn) {
            camera_pos = camera_pos - 0.05*cam_roll_axis;
            camera_lookat = camera_lookat - 0.05*cam_roll_axis;
        }
        if (fTransYp) {
            // camera_pos = camera_pos + 0.05*cam_lookat_axis;
            camera_pos = camera_pos + 0.05*cam_up_axis;
            camera_lookat = camera_lookat + 0.05*cam_up_axis;
        }
        if (fTransYn) {
            // camera_pos = camera_pos - 0.05*cam_lookat_axis;
            camera_pos = camera_pos - 0.05*cam_up_axis;
            camera_lookat = camera_lookat - 0.05*cam_up_axis;
        }
        if (fTransZp) {
            camera_pos = camera_pos + 0.1*cam_depth_axis;
            camera_lookat = camera_lookat + 0.1*cam_depth_axis;
        }
        if (fTransZn) {
            camera_pos = camera_pos - 0.1*cam_depth_axis;
            camera_lookat = camera_lookat - 0.1*cam_depth_axis;
        }
        if (fRotPanTilt) {
            // get current cursor position
            double cursorx, cursory;
            glfwGetCursorPos(window, &cursorx, &cursory);
            //TODO: might need to re-scale from screen units to physical units
            double compass = 0.006*(cursorx - last_cursorx);
            double azimuth = 0.006*(cursory - last_cursory);
            double radius = (camera_pos - camera_lookat).norm();
            Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
            camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
            Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
            camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
        }
        graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
        glfwGetCursorPos(window, &last_cursorx, &last_cursory);
        
        count++;
    }
    

    // wait for simulation to finish
    fSimulationRunning = false;
    sim_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

    return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
    // prepare simulation
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    VectorXd g = VectorXd::Zero(dof);
    string controller_status = "0";
    double kv = 10;  // can be set to 0 if no damping is needed
    VectorXd panda_q(7), panda_dq(7), gripper_q(2), gripper_dq(2);

    // setup redis callback
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

    // add to read callback
    redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
    redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // add to write callback
    redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, panda_q);
    redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, panda_dq);
    redis_client.addEigenToWriteCallback(0, GRIPPER_JOINT_ANGLES_KEY, gripper_q);
    redis_client.addEigenToWriteCallback(0, GRIPPER_JOINT_VELOCITIES_KEY, gripper_dq);

    Eigen::VectorXd dart_torques = VectorXd::Zero(6);
    dart_torques(2) = 0.12 * 9.81;

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    bool fTimerDidSleep = true;
    double start_time = timer.elapsedTime();
    double last_time = start_time;
    auto testString = redis_client.get(TEST_KEY_STRING);
    auto target = redis_client.get(BOARD_TARGET);

    // start simulation
    fSimulationRunning = true;
    while (fSimulationRunning) {
        fTimerDidSleep = timer.waitForNextLoop();

        // execute redis read callback
        redis_client.executeReadCallback(0);

        // apply gravity compensation
        robot->gravityVector(g);

        // set joint torques
        if (controller_status == "1") {
            sim->setJointTorques(robot_name, command_torques + g);
            //sim->setJointTorques(object_names[0], dart_torques);
        } else {
            sim->setJointTorques(robot_name, g - robot->_M * (kv * robot->_dq));
        }

        // integrate forward
        double curr_time = timer.elapsedTime();
        double loop_dt = curr_time - last_time;
        sim->integrate(0.001);

        // read joint positions, velocities, update model
        sim->getJointPositions(robot_name, robot->_q);
        sim->getJointVelocities(robot_name, robot->_dq);
        robot->updateModel();

        panda_q = robot->_q.head(7);
        panda_dq = robot->_dq.head(7);
        gripper_q = robot->_q.tail(2);
        gripper_dq = robot->_dq.tail(2);

        // get dynamic object positions
        if ((q_launch-robot->_q).norm() < 0.35){
            lol = true;
        }
        if (lol){
            if ((q_launch-robot->_q).norm() > 0.5){
                cout << "yooo" << endl;
                // sim->setCoeffFrictionStatic(0);
                // sim->setCoeffFrictionDynamic(0);
            }
        }
		cout << "position" << endl;
		cout << object_pos[0] << endl;
        if (hit == false){
            //object_pos[0](2) += 0.001;
            //sim->setObjectPosition(object_names[0], object_pos[0], object_ori[0]);
            for (int i = 0; i < n_objects; ++i) {
                sim->getObjectPosition(object_names[i], object_pos[i], object_ori[i]);
                sim->getObjectVelocity(object_names[i], object_lin_vel[i], object_ang_vel[i]);

                if (object_lin_vel[i](2) > 2 && curr_time > 10){
                    bd = true; 
                }
                if (bd == true){
					object_ori[0] = Quaterniond(0.5, 0.5, -0.5, 0.5);
					cout << "bd" << endl;
                    if (release==false){
                        //if (object_pos[0](2) > 1.09 && object_pos[0](2) < 1.1){
                        if (testString == "Released !!!"){
                            release = true;
                            if(target == "4"){
								vy = sqrt((4*9.81)/(2*sqrt(3)));
								vz = sqrt(3)*vy;
                                // object_lin_vel[0](1) = sqrt(9.81*2);
                                // object_lin_vel[0](2) = sqrt(9.81*2);
                            }
                            else if(target == "3"){
								vy = sqrt((3*9.81)/(2*sqrt(3)));
								vz = sqrt(3)*vy;
                                // object_lin_vel[0](1) = sqrt(9.81*1.5);
                                // object_lin_vel[0](2) = sqrt(9.81*1.5);
                            }
                            else if(target == "2"){
								vy = sqrt((2*9.81)/(2*sqrt(3)));
								vz = sqrt(3)*vy;
                                // object_lin_vel[0](1) = sqrt(9.81);
                                // object_lin_vel[0](2) = sqrt(9.81);
                            }
                            else if(target == "1"){
								vy = sqrt(9.81/(2*sqrt(3)));
								vz = sqrt(3)*vy;
                                // object_lin_vel[0](1) = sqrt(9.81/2);
                                // object_lin_vel[0](2) = sqrt(9.81/2);
                            }
                            sim->setCoeffFrictionStatic(0);
                            sim->setCoeffFrictionDynamic(0);
							// object_pos[0](0) = 0; //artificially moving dart into correct launch position
                            // object_pos[0](1) = 0; // "
                            // object_pos[0](2) = 1.1; // "
                            // object_lin_vel[0](0) = 0;  // artificially giving dart an initial velocity
							// object_lin_vel[0](1) = 0;
							// object_lin_vel[0](2) = 0;
                            // object_ori[0] = Quaterniond(0.5, 0.5, -0.5, 0.5); //make dart parallel with ground
							// sim->setObjectPosition(object_names[0], object_pos[0], object_ori[0]);
                            
                            // Now we've given the dart a position, orientation and velocity - the simviz handles updating its position as it flies to the board
                        }
                    } else {
						// object_ori[0] = Quaterniond(0.5, 0.5, -0.5, 0.5); //make dart parallel with ground
						// cout << object_ori[0] << endl;
						// sim->setObjectPosition(object_names[0], object_pos[0], object_ori[0]);
						// object_pos[0](0) += 0;
						// object_pos[0](1) += time2*vyz;
						// object_pos[0](2) += time2*vyz - 0.5*9.81*time2*time2;
						// object_lin_vel[0](0) += 0;
						// object_lin_vel[0](1) += 0;
						// object_lin_vel[0](2) += -time2*9.81;
						object_ori[0] = Quaterniond(0.5, 0.5, -0.5, 0.5);
						sim->setObjectPosition(object_names[0], object_pos[0], object_ori[0]);
						time2 += 0.001;
						cout << "time" << endl;
						cout << time2 << endl;
						cout << "v" << endl;
						cout << object_lin_vel[0] << endl;
						cout << "position" << endl;
						cout << object_pos[0] << endl;
					}
                    
                    if (object_pos[0](1) > 0.965 && target == "1"){
                        hit = true;
                        dart_hit(0) = object_pos[i](0);
                        dart_hit(1) = object_pos[i](1);
                        dart_hit(2) = object_pos[i](2);  //1.1;  // this is Kyle's hacky way to ensure bullseye
                        break;
                    }
                    if (object_pos[0](1) > 1.965 && target == "2"){
                        hit = true;
                        dart_hit(0) = object_pos[i](0);
                        dart_hit(1) = object_pos[i](1);
                        dart_hit(2) = object_pos[i](2);  //1.1;  // this is Kyle's hacky way to ensure bullseye
                        break;
                    }
                    if (object_pos[0](1) > 2.965 && target == "3"){
                        hit = true;
                        dart_hit(0) = object_pos[i](0);
                        dart_hit(1) = object_pos[i](1);
                        dart_hit(2) = object_pos[i](2);  //1.1;  // this is Kyle's hacky way to ensure bullseye
                        break;
                    }
                    if (object_pos[0](1) > 3.965 && target == "4"){
                        hit = true;
                        dart_hit(0) = object_pos[i](0);
                        dart_hit(1) = object_pos[i](1);
                        dart_hit(2) = object_pos[i](2);  //1.1;  // this is Kyle's hacky way to ensure bullseye
                        break;
                    }
                }
            }
        } else {
            object_pos[0](0) = dart_hit(0);
            object_pos[0](1) = dart_hit(1);
            object_pos[0](2) = dart_hit(2); //1.2;
			object_lin_vel[0](0) = 0;  // artificially giving dart an initial velocity
			object_lin_vel[0](1) = 0;
			object_lin_vel[0](2) = 0;
            object_ori[0] = Quaterniond(0.5, 0.5, -0.5, 0.5);
			sim->setObjectPosition(object_names[0], object_pos[0], object_ori[0]);
        }
        
        testString = redis_client.get(TEST_KEY_STRING);
        if (testString == "Running Controller"){
            cout << target << "\n";
        }
        
        //cout << testString << "\n";

        // execute redis write callback
        redis_client.executeWriteCallback(0);

        // update last time
        last_time = curr_time;
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
    cerr << "GLFW Error: " << description << endl;
    exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
    bool ret = false;
    #ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK) {
        cout << "Failed to initialize GLEW library" << endl;
        cout << glewGetErrorString(ret) << endl;
        glfwTerminate();
    } else {
        ret = true;
    }
    #endif
    return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
        case GLFW_KEY_ESCAPE:
            // exit application
            fSimulationRunning = false;
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;
        case GLFW_KEY_RIGHT:
            fTransXp = set;
            break;
        case GLFW_KEY_LEFT:
            fTransXn = set;
            break;
        case GLFW_KEY_UP:
            fTransYp = set;
            break;
        case GLFW_KEY_DOWN:
            fTransYn = set;
            break;
        case GLFW_KEY_A:
            fTransZp = set;
            break;
        case GLFW_KEY_Z:
            fTransZn = set;
            break;
        default:
            break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
    bool set = (action != GLFW_RELEASE);
    //TODO: mouse interaction with robot
    switch (button) {
        // left click pans and tilts
        case GLFW_MOUSE_BUTTON_LEFT:
            fRotPanTilt = set;
            // NOTE: the code below is recommended but doesn't work well
            // if (fRotPanTilt) {
            //     // lock cursor
            //     glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            // } else {
            //     glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            // }
            break;
        // if right click: don't handle. this is for menu selection
        case GLFW_MOUSE_BUTTON_RIGHT:
            fRobotLinkSelect = set;
            break;
        // if middle click: don't handle. doesn't work well on laptops
        case GLFW_MOUSE_BUTTON_MIDDLE:
            break;
        default:
            break;
    }
}


