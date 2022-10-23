
#include <../my_utils/Configuration.h>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>

#include <dirent.h>
#include <dlfcn.h>
#include <sys/errno.h>
#include <unistd.h>

#include <my_simulator/Mujoco/mujoco_sim_utils.h>
#include <my_robot_core/anymal_core/anymal_interface.hpp>


// -------- static variables -------- //
// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;

SimulationConfiguration* simConfig = new SimulationConfiguration();
bool b_initialize = false;


void InitializeConfiguration(){

  // Initialize && Parameter setting

  // base
  for(int i(0); i<7; ++i)
    d->qpos[i]=simConfig->initial_pose[i];

  //leg
  std::array<double,4> sign1 = {1,-1,1,-1};
  std::array<double,4> sign2 = {1,1,-1,-1};
  for(int i=0; i< 4; ++i){
    d->qpos[7+3*i] = sign1[i]*simConfig->initial_leg_config[0];
    d->qpos[7+3*i+1] = sign2[i]*simConfig->initial_leg_config[1];
    d->qpos[7+3*i+2] = -sign2[i]*simConfig->initial_leg_config[2];
  }

  //arm
  for(int i(0); i<6; ++i)
    d->qpos[19+i]=simConfig->initial_arm_config[i];

  // set joints config
  simConfig->initial_joints_config = Eigen::VectorXd::Zero(m->nu);
  for(int i(0); i<m->nu; ++i){
    simConfig->initial_joints_config[i] = d->qpos[7+i];
  }  

  std::cout<<" ==== INITIALIZE CONFIGURATION ==== "<< std::endl;
  mujoco_utils::print_qpos(m,d);
}

void PhysicsLoop(mujoco::Simulate& sim) {
  // cpu-sim syncronization point
  double syncCPU = 0;
  mjtNum syncSim = 0;

  // ---- SET INTERFACE
  ANYmalInterface* interface_ = new ANYmalInterface();
  ANYmalSensorData* sensor_data_ = new ANYmalSensorData();
  ANYmalCommand* command_ = new ANYmalCommand();
  Eigen::VectorXd trq_cmd_ = Eigen::VectorXd::Zero(ANYmal::n_dof);

  // run until asked to exit
  while (!sim.exitrequest.load()) {

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if(sim.motionrequest.load()){
      std::cout<<"motionrequest loaded "<<std::endl;
      interface_ -> interrupt_ -> setFlags('s');  
      sim.motionrequest.store(false);
    }

    {
      // lock the sim mutex
      const std::lock_guard<std::mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          // record cpu time at start of iteration
          double startCPU = mujoco::Glfw().glfwGetTime();

          if(syncCPU==0){
              syncCPU = startCPU;
              syncSim = d->time;
          }

          // elapsed CPU and simulation time since last sync
          double elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;  

          // update sensor data
          sensor_data_->elapsedtime = elapsedSim;
          sensor_data_->q.setZero();
          sensor_data_->qdot.setZero();
          
          for(int i=0; i< (m->nu); ++i) {
            sensor_data_->q[i] = d->qpos[7+i]; 
            sensor_data_->qdot[i] = d->qvel[6+i]; 
            sensor_data_->tau_cmd_prev[i] = 0.; //trq_cmd_[i];
          }

          for(int i=0; i< 7; ++i) { //3(xyz) + 4(quat:w,x,y,z)
            sensor_data_->virtual_q[i] = d->qpos[i]; } 
          for(int i=0; i< 6; ++i) { //3(xyz) + 3(wx,wy,wz) 
            sensor_data_->virtual_qdot[i] = d->qvel[i]; }  

          // std::cout<<"base ori vel: " <<
          //     d->qvel[3] << ", " <<
          //     d->qvel[4] << ", " <<
          //     d->qvel[5] << std::endl;          
            

          if( elapsedSim < 1.0 ){
            // interface_->getCommand(sensor_data_, command_);       
            // std::cout<<" (elapsedSim<1) d->time = " <<d->time<<", elapsedSim = " << elapsedSim << std::endl;
            for (int i=0; i<m->nu; i++) {
              command_->q[i] = simConfig->initial_joints_config[i];
              command_->qdot[i] = 0.0;
            }
            
          }
          else{
            // std::cout<<" (elapsedSim>1) d->time = " <<d->time<<", elapsedSim = " << elapsedSim << std::endl;
            static bool initdone = true;
            if(initdone){
              std::cout<<" start getCommand() " << std::endl;
              initdone = false;
            }
            interface_->getCommand(sensor_data_, command_);            
          }

          // my_utils::pretty_print(command_->jtrq, std::cout, "cmd jtrq");
          // my_utils::pretty_print(command_->q, std::cout, "cmd q");
          // my_utils::pretty_print(command_->qdot, std::cout, "cmd qdot");
          // my_utils::pretty_print(sensor_data_->q, std::cout, "sen q");
          // my_utils::pretty_print(sensor_data_->qdot, std::cout, "sen qdot");

          Eigen::VectorXd tau_fb = Eigen::VectorXd::Zero(m->nu);
          for (int i=0; i<m->nu; i++) {
            tau_fb[i] = simConfig->Kp[i] * (command_->q[i] - sensor_data_->q[i])
                        + simConfig->Kd[i] * (command_->qdot[i] - sensor_data_->qdot[i]);
            
            d->ctrl[i] = std::max(tau_fb[i], -simConfig->trq_limit);
            d->ctrl[i] = std::min(d->ctrl[i], +simConfig->trq_limit);

            tau_fb[i] = d->ctrl[i]; // + command_->jtrq[i]
          }          
          // my_utils::pretty_print(tau_fb, std::cout, "tau");
          // std::cout<<"------------------------------------------"<<std::endl;

          // clear old perturbations, apply new
          mju_zero(d->xfrc_applied, 6*m->nbody);
          sim.applyposepertubations(0);  // move mocap bodies only
          sim.applyforceperturbations();

          // run single step, let next iteration deal with timing
          mj_step(m, d);         
          
        }
        // paused
        else {
          // apply pose perturbation
          sim.applyposepertubations(1);  // move mocap and dynamic bodies

          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
        }
      }
    }  // release std::lock_guard<std::mutex>
  }

  delete interface_;
  delete sensor_data_;
  delete command_;
}

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mujoco::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    char loadError[kErrorLength] = "";
    m = mj_loadXML(filename, nullptr, loadError, mujoco::Simulate::kMaxFilenameLength);
    if (m) d = mj_makeData(m);
    if (d) {
      sim->load(filename, m, d, true);

      // d -> set robot init config
      InitializeConfiguration();

      mj_step(m, d);      
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

int main(int argc, char** argv) {

    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER!=mj_version()) {
        mju_error("Headers and library have different versions");
    }

    std::vector<unique_dlhandle> dso_handles = scanPluginLibraries();

    // simulate object encapsulates the UI
    auto sim = std::make_unique<mujoco::Simulate>();

    // init GLFW
    if (!mujoco::Glfw().glfwInit()) {
      mju_error("could not initialize GLFW");
    }

    const char* filename = nullptr;
    if (argc >  1) {
        filename = argv[1];
    }
    else{
        filename = THIS_COM "/robot_description/Robot/ANYmalwithArm/scene_anymal_c_ur3.xml";
    }

    std::thread physicsthreadhandle = std::thread(&PhysicsThread, sim.get(), filename);

    std::cout<<"hi 1 "<< std::endl;
    GLFWmonitor* mon = mujoco::Glfw().glfwGetPrimaryMonitor();
    std::cout<<mon<<std::endl;
    // std::cout<<*mujoco::Glfw().glfwGetVideoMode(mon)<<std::endl;

    sim->renderloop();
    physicsthreadhandle.join();

    return 0;
    // m = mj_loadXML(THIS_COM "src/my_simulator/src/Mujoco/ANYmalC/anybotics_anymal_c/scene.xml", nullptr, nullptr, 0);
    // d = mj_makeData(m);

    // std::cout<<"hi 1 "<< std::endl;

    // MujocoSimulationNode* mj_sim_node = new MujocoSimulationNode();
    // mj_step(mj_sim::model, mj_sim::data);

    // std::cout<<"hi 1 "<< std::endl;

    // GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    // mj_sim_node->runInit(window);

    // std::cout<<"hi 1 "<< std::endl;


    
    // // run main loop, target real-time simulation and 60 fps rendering
    // while (!glfwWindowShouldClose(window)) {

    //     mj_sim_node->oneStep(window);
    // }
}