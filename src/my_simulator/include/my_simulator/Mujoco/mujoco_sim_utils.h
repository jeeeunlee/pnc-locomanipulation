#pragma once

#include <stdio.h>
#include <iostream>
#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <cstring>

#include <mujoco.h>
#include <GLFW/glfw3.h>
#include <ThirdParty/mujoco_simulate/simulate.h>
#include <ThirdParty/mujoco_simulate/glfw_dispatch.h>

#include <my_utils/IO/IOUtilities.hpp>

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


class SimulationConfiguration{
  public:
    SimulationConfiguration(){
      YAML::Node simulation_cfg;
      try {
          simulation_cfg =
              YAML::LoadFile(THIS_COM "config/ANYmal/SIMULATION_MUJOCO.yaml");

          my_utils::readParameter(simulation_cfg, "initial_pose", initial_pose);   
          my_utils::readParameter(simulation_cfg, "initial_leg_config", initial_leg_config);
          my_utils::readParameter(simulation_cfg, "initial_arm_config", initial_arm_config);

          my_utils::readParameter(simulation_cfg["control_configuration"], "kp", Kp);  
          my_utils::readParameter(simulation_cfg["control_configuration"], "kd", Kd);
          my_utils::readParameter(simulation_cfg["control_configuration"], "torque_limit", trq_limit);
                
      } catch (std::runtime_error& e) {
          std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                    << __FILE__ << "]" << std::endl
                    << std::endl;
      }
    }

  public:
    Eigen::VectorXd initial_pose;
    Eigen::VectorXd initial_leg_config;
    Eigen::VectorXd initial_joints_config;
    Eigen::VectorXd initial_arm_config;

    Eigen::VectorXd Kp;
    Eigen::VectorXd Kd;
    // Eigen::VectorXd trq_limit;
    double trq_limit;
};


namespace mujoco_utils{

void print_ctrl(const mjModel* m, mjData* d){
  std::cout <<" ctrl : ";
  for (int i=0; i<m->nu; i++) {
    std::cout << d->ctrl[i] << ", ";
  }
  std::cout<<std::endl;
}

void print_qpos(const mjModel* m, mjData* d){
  std::cout <<" qpos : ";
  for (int i=0; i<m->nq; i++) {
    std::cout << d->qpos[i] << ", ";
  }
  std::cout<<std::endl;
}

void print_qvel(const mjModel* m, mjData* d){
  std::cout <<" qvel : ";
  for (int i=0; i<m->nq; i++) {
    std::cout << d->qvel[i] << ", ";
  }
  std::cout<<std::endl;
}

};
  