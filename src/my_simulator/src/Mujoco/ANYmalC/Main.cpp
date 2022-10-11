
#include <../my_utils/Configuration.h>
#include <stdio.h>
#include <iostream>
#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <cstring>

#include <dirent.h>
#include <dlfcn.h>
#include <sys/errno.h>
#include <unistd.h>

#include <mujoco.h>
#include <GLFW/glfw3.h>
#include <ThirdParty/mujoco_simulate/simulate.h>
#include <ThirdParty/mujoco_simulate/glfw_dispatch.h>

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


using unique_dlhandle = std::unique_ptr<void, decltype(&dlclose)>;

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {

  constexpr char kPathSep = '/';
  const char* path = "/proc/self/exe";

  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();


  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



std::vector<unique_dlhandle> scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

    const std::string sep = "/";
    const std::string dso_suffix = ".so";

  // output vectors containing DSO handles
  std::vector<unique_dlhandle> dso_handles;

  // platform-independent routine for checking and printing plugins registered by a dynamic library
  const auto check_and_print_plugins = [&](const std::string& name, unique_dlhandle&& dlhandle) {
    if (!dlhandle) {
      return;
    }

    const int nplugin_new = mjp_pluginCount();
    if (nplugin_new > nplugin) {
      dso_handles.push_back(std::move(dlhandle));

      // print all newly registered plugins
      std::printf("Plugins registered by library '%s':\n", name.c_str());
      for (int i = nplugin; i < nplugin_new; ++i) {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }

      // update counter for plugins registered so far
      nplugin = nplugin_new;
    }
  };

  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return dso_handles;
  }

  const std::string plugin_dir = getExecutableDir() + sep + "plugin";

    DIR* dirp = opendir(plugin_dir.c_str());
  if (!dirp) {
    return dso_handles;
  }

  // go through each entry in the directory
  for (struct dirent* dp; (dp = readdir(dirp));) {
    // only look at regular files (i.e. skip symlinks, pipes, directories, etc.)
    if (dp->d_type == DT_REG) {
      const std::string name(dp->d_name);
      if (name.size() > dso_suffix.size() &&
          name.substr(name.size() - dso_suffix.size()) == dso_suffix) {
        // load the library and check for plugins
        const std::string dso_path = plugin_dir + sep + name;
        check_and_print_plugins(
            name, unique_dlhandle(dlopen(dso_path.c_str(), RTLD_NOW | RTLD_LOCAL), &dlclose));
      }
    }
  }
  closedir(dirp);

  return dso_handles;
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

          // elapsed CPU and simulation time since last sync
          double elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // static int count = 0;
          // if(count++ > 10)
          //   exit(0);

          // std::cout<< " m->nu = " << m->nu << std::endl;
          // std::cout<< " m->nq = " << m->nq << std::endl;
          // std::cout<< " m->na = " << m->na << std::endl;
          // std::cout<< " m->nbody = " << m->nbody << std::endl;

          // std::cout<< " body = ";
          // for(int i=0; i<m->nbody; i++) {
          //   std::cout<< m->names[ m->name_bodyadr[i] ] << " = ";
          //   std::cout<< d->xpos[3*i] << ", " << d->xpos[3*i] << ", " << d->xpos[3*i] << std::endl;
          //   std::cout<< d->cvel[6*i+3] << ", " << d->cvel[6*i+4] << ", " << d->cvel[6*i+5] << std::endl;
          // }

          // std::cout<<"elapsedSim = "<<elapsedSim<<std::endl;

          int nBase = 7;
          sensor_data_->q.setZero();
          sensor_data_->qdot.setZero();
          for(int i=0; i< (m->nq-nBase); ++i) {
            sensor_data_->q[i] = d->qpos[nBase+i];
            sensor_data_->qdot[i] = d->qvel[nBase+i];
            sensor_data_->tau_cmd_prev[i] = 0.; //trq_cmd_[i];
          }

          sensor_data_->q[12]=0.2;
          sensor_data_->q[13]= -2.0; 
          sensor_data_->q[14]= 1.5;
          sensor_data_->q[15]= -1.0;
          sensor_data_->q[16]= -1.2;
          sensor_data_->q[17]= 0.0;


          int nbodybase = 1; // world:0, base:1
          for(int i=0; i< 3; ++i) {
            sensor_data_->virtual_q[i] = d->xpos[3*nbodybase + i];
            sensor_data_->virtual_qdot[i] = d->cvel[6*nbodybase+3+i];
          }

          interface_->getCommand(sensor_data_, command_);

          // std::cout<< " sensor_data_->virtual_q = ";
          // for(int i=0; i<ANYmal::n_vdof; i++){
          //   std::cout<<  sensor_data_->virtual_q[i] << ", ";
          // }
          // std::cout<<std::endl;

          // std::cout<< " sensor_data_->virtual_qdot = ";
          // for(int i=0; i<ANYmal::n_vdof; i++){
          //   std::cout<<  sensor_data_->virtual_qdot[i] << ", ";
          // }
          // std::cout<<std::endl;

          // std::cout<< " command_->qdot = ";
          // for(int i=0; i<ANYmal::n_adof; i++){
          //   std::cout<<  command_->qdot[i] << ", ";
          // }
          // std::cout<<std::endl;

          // std::cout<< " sensor_data_->qdot = ";
          // for(int i=0; i<ANYmal::n_adof; i++){
          //   std::cout<<  sensor_data_->qdot[i]<< ", ";
          // }
          // std::cout<<std::endl;

          // std::cout<< " command_->q = ";
          // for(int i=0; i<ANYmal::n_adof; i++){
          //   std::cout<<  command_->q[i] << ", ";
          // }
          // std::cout<<std::endl;

          // std::cout<< " sensor_data_->q = ";
          // for(int i=0; i<ANYmal::n_adof; i++){
          //   std::cout<<  sensor_data_->q[i] << ", ";
          // }
          // std::cout<<std::endl;
                    

          std::cout<<"Trq = ";
          for (int i=0; i<m->nu; i++) {
            d->ctrl[i] = 0.0;
            // 0.01 * (command_->q[i] - sensor_data_->q[i]);
            //0.001 * (command_->qdot[i] - sensor_data_->qdot[i]) +
            std::cout<<d->ctrl[i]<<", ";
          }
          std::cout<<std::endl;

          // for (int i=0; i< (m->nq-nBase); ++i) {
          //   d->qpos[nBase+i]=command_->q[i]; 
          //   d->qvel[nBase+i]=command_->qdot[i];
          // }

          


          // inject noise
          if (sim.ctrlnoisestd) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep / sim.ctrlnoiserate);
            mjtNum scale = sim.ctrlnoisestd * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] += ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.realTimeIndex];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned = mju_abs(elapsedCPU/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU < 0 || syncCPU == 0 || misaligned || sim.speedChanged) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speedChanged = false;

            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6*m->nbody);
            sim.applyposepertubations(0);  // move mocap bodies only
            sim.applyforceperturbations();

            // run single step, let next iteration deal with timing
            mj_step(m, d);
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;
            double refreshTime = simRefreshFraction/sim.refreshRate;

            // step while sim lags behind cpu and within refreshTime
            while ((d->time - syncSim)*slowdown < (mujoco::Glfw().glfwGetTime()-syncCPU) &&
                   (mujoco::Glfw().glfwGetTime()-startCPU) < refreshTime) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measuredSlowdown = elapsedCPU / elapsedSim;
                measured = true;
              }

              // clear old perturbations, apply new
              mju_zero(d->xfrc_applied, 6*m->nbody);
              sim.applyposepertubations(0);  // move mocap bodies only
              sim.applyforceperturbations();

              // call mj_step
              mj_step(m, d);

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }
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

      // Initialize && Parameter setting

      d->qpos[2]=0.52;
      std::array<double,4> sign1 = {1,-1,1,-1};
      std::array<double,4> sign2 = {1,1,-1,-1};
      for(int i=0; i< 4; ++i){
        d->qpos[7+3*i] = sign1[i]*0.1;
        d->qpos[7+3*i+1] = sign2[i]*0.6;
        d->qpos[7+3*i+2] = -sign2[i]*1.0;
      }
      
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
        filename = THIS_COM "src/my_simulator/src/Mujoco/ANYmalC/anybotics_anymal_c/scene.xml";
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