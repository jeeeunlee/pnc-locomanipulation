#pragma once

#include <my_wbc/Task/Task.hpp>

class RobotSystem;

class CoMTask : public Task {
   public:
    CoMTask(RobotSystem*);
    virtual ~CoMTask();

   protected:
    /* Update pos_err, vel_des, acc_des
     *
     * pos_des_ = [quat_w, quat_x, quat_y, quat_z, x, y, z]
     * vel_des_ = [w_x, w_y, w_z, x_dot, y_dot, z_dot]
     * acc_des_ = [a_x, a_y, a_z, x_ddot, y_ddot, z_ddot]
     */

    double control_period;

    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des);
    virtual bool _UpdateTaskJacobian();
    virtual bool _UpdateTaskJDotQdot();
};
