#pragma once
#include <Eigen/Dense>

// 2021.1.26. test
// trajectory parameterization
// TRY #1
// task 1 : foot traj
// task 2 : joint traj


#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_pnc/MagnetoPnC/MagnetoTask/TaskSet.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>

class MagnetoControlArchitecture;

class MagnetoTrajectoryManager {
    public:
        MagnetoTrajectoryManager(MagnetoControlArchitecture* _ctrl_arch);
        ~MagnetoTrajectoryManager();
        bool ParameterizeTrajectory(MotionCommand& motion_cmd,
                                    const double& x_ratio_height, 
                                    const double& t_holdstart, 
                                    const double& t_swing,
                                    const double& t_holdend);

        void update(const double& curr_time,
                    Eigen::VectorXd& q,
                    Eigen::VectorXd& dotq,
                    Eigen::VectorXd& ddotq,
                    bool& is_swing);
        void updateContact(int moving_foot_idx);

        

    protected:
        MagnetoControlArchitecture* ctrl_arch_;
        RobotSystem* robot_manager_;
        KinWBC* kin_wbc_;

        std::vector<Task*> task_list_;
        std::vector<ContactSpec*> contact_list_;

        std::map<int, Task*> foot_task_map_;
        std::map<int, ContactSpec*> foot_contact_map_;

        ContactSpec* alfoot_contact_;
        ContactSpec* arfoot_contact_;
        ContactSpec* blfoot_contact_;
        ContactSpec* brfoot_contact_;

        Task* alfoot_pos_task_;
        Task* arfoot_pos_task_;
        Task* blfoot_pos_task_;
        Task* brfoot_pos_task_;

        Task* foot_pos_task_;
        Task* joint_task_;


    private:
        int n_dim_ ;

        Eigen::VectorXd q_init_;
        Eigen::VectorXd q_goal_;
        Eigen::VectorXd dotq_init_; 
        Eigen::VectorXd dotq_goal_;

         int moving_foot_idx_;
        MOTION_DATA foot_motion_data_;

        double t0_;
        double t1_;
        double t2_;
        double t3_;

};

