
#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoTrajectoryManager.hpp>

MagnetoTrajectoryManager::MagnetoTrajectoryManager(MagnetoControlArchitecture* _ctrl_arch) {
    my_utils::pretty_constructor(2, "Magneto Trajectory Planner");

    ctrl_arch_ = _ctrl_arch;    

    robot_manager_ = new RobotSystem(*(ctrl_arch_->robot_));

    std::vector<bool> act_list;
    act_list.resize(Magneto::n_dof, true);
    for (int i(0); i < Magneto::n_vdof; ++i) 
        act_list[Magneto::idx_vdof[i]] = false;

    // Initialize WBC
    kin_wbc_ = new KinWBC(act_list);

    // contact
    double mu_ = 0.7; // will be updated later
    alfoot_contact_ = new BodyFramePointContactSpec(robot_manager_,
                                MagnetoBodyNode::AL_foot_link, mu_);
    blfoot_contact_ = new BodyFramePointContactSpec(robot_manager_,
                                MagnetoBodyNode::BL_foot_link, mu_);                          
    arfoot_contact_ = new BodyFramePointContactSpec(robot_manager_,
                                MagnetoBodyNode::AR_foot_link, mu_);
    brfoot_contact_ = new BodyFramePointContactSpec(robot_manager_,
                                MagnetoBodyNode::BR_foot_link, mu_);

    foot_contact_map_ = {
        {MagnetoBodyNode::AL_foot_link,  alfoot_contact_},
        {MagnetoBodyNode::AR_foot_link,  blfoot_contact_},
        {MagnetoBodyNode::BL_foot_link,  arfoot_contact_},
        {MagnetoBodyNode::BR_foot_link,  brfoot_contact_}
    };

    joint_task_ = 
      new BasicTask(robot_manager_, BasicTaskType::FULLJOINT, Magneto::n_adof);

    // Set Foot Motion Tasks
    alfoot_pos_task_ =
        new BasicTask(robot_manager_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::AL_foot_link);
    arfoot_pos_task_ =
        new BasicTask(robot_manager_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::AR_foot_link);
    blfoot_pos_task_ =
        new BasicTask(robot_manager_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::BL_foot_link);
    brfoot_pos_task_ =
        new BasicTask(robot_manager_, BasicTaskType::LINKXYZ, 3, MagnetoBodyNode::BR_foot_link);

    foot_task_map_ = { 
        {MagnetoBodyNode::AL_foot_link,  alfoot_pos_task_},
        {MagnetoBodyNode::AR_foot_link,  arfoot_pos_task_},
        {MagnetoBodyNode::BL_foot_link,  blfoot_pos_task_},
        {MagnetoBodyNode::BR_foot_link,  brfoot_pos_task_}
    };
}

MagnetoTrajectoryManager::~MagnetoTrajectoryManager() {}

bool MagnetoTrajectoryManager::ParameterizeTrajectory(MotionCommand& motion_cmd,
                                                    const double& x_ratio_height, 
                                                    const double& t_starthold, 
                                                    const double& t_swing,
                                                    const double& t_endhold){
    
    if( !motion_cmd.get_foot_motion_command(foot_motion_data_, moving_foot_idx_) )
        return false;    

    // get init config and goal config                                                        
    n_dim_ = ctrl_arch_->robot_->getNumDofs();
    q_init_ = ctrl_arch_->robot_->getQ();
    dotq_init_ = ctrl_arch_->robot_->getQdot(); 
    ctrl_arch_->goal_planner_->computeGoal(motion_cmd);
    ctrl_arch_->goal_planner_->getGoalConfiguration(q_goal_);
    dotq_goal_ = Eigen::VectorXd::Zero(n_dim_);
    robot_manager_->updateSystem(q_init_, dotq_init_, false);    

    t0_ = 0.0; // step start (move under full support)
    t1_ = t0_ + t_starthold; // swing start
    t2_ = t1_ + t_swing; // full support start
    t3_ = t2_ + t_endhold; // end step

    // 0. contact

    // 1. swing foot trajectory
    foot_motion_data_.motion_period = t_swing;    
    motion_cmd.clear_and_add_motion(moving_foot_idx_,foot_motion_data_);
    foot_pos_task_ = foot_task_map_[moving_foot_idx_];


    ctrl_arch_->foot_trajectory_manager_
                ->setFootPosTrajectory(t1_ , &motion_cmd, x_ratio_height);
    // 2. joint trajectory
    ctrl_arch_->joint_trajectory_manager_
                ->setJointTrajectory(t0_, t3_, q_goal_);

    return true;
    
}

void MagnetoTrajectoryManager::update(const double& curr_time,
                                      Eigen::VectorXd& q,
                                      Eigen::VectorXd& dotq,
                                      Eigen::VectorXd& ddotq,
                                      bool is_swing) {

    std::cout<<" here : curr_time=" << curr_time <<  std::endl;
    std::cout<<" t0 = " << t0_ <<" t1 = " << t1_ << ", t2 = " << t2_ << ", t3 = " << t3_ << std::endl;
    // update task
    task_list_.clear();
    if(curr_time < t0_){ // before start
        q = q_init_;
        dotq = dotq_init_;
        return;
    }else if(curr_time < t1_){ // full support
        ctrl_arch_->joint_trajectory_manager_->updateTask(curr_time, joint_task_);
        task_list_.push_back(joint_task_);
        updateContact(-1); //full contact
    }else if(curr_time < t2_){ // swing
        ctrl_arch_->foot_trajectory_manager_->updateTask(curr_time, foot_pos_task_); 
        ctrl_arch_->joint_trajectory_manager_->updateTask(curr_time, joint_task_);
        task_list_.push_back(foot_pos_task_);
        task_list_.push_back(joint_task_);
        updateContact(moving_foot_idx_); // swing contact
    }else if(curr_time < t3_){ // support
        ctrl_arch_->joint_trajectory_manager_->updateTask(curr_time, joint_task_);
        task_list_.push_back(joint_task_);
        updateContact(-1); //full contact
    }else{ // after end
        q = q_goal_;
        dotq = dotq_goal_;
        return;
    }

    std::cout<<"here 1 : contact=" << contact_list_.size() << ", task=" << task_list_.size() << std::endl;
    Eigen::VectorXd q_curr = robot_manager_->getQ();
    if(!task_list_.empty()) {
          kin_wbc_->FindFullConfiguration(q_curr, task_list_, contact_list_, 
                                        q, dotq, jacc_des_); 
    }
    Eigen::VectorXd f_err = foot_pos_task_->pos_err;
    my_utils::pretty_print(f_err, std::cout, "f_err");
    std::cout<<"here 2 " << std::endl;

    robot_manager_->updateSystem(q, dotq, false);
    
}

// void MagnetoTrajectoryManager::updateDdotQ(const Eigen::VectorXd& dotq_des_next,
//                                         const Eigen::VectorXd& dotq_des,
//                                         const Eigen::VectorXd& q_des,
//                                         const Eigen::VectorXd& dotq,
//                                         const Eigen::VectorXd& q,
//                                         const double& timestep,
//                                         Eigen::VectorXd& ddotq){

//     ddotq = 1./timestep * (dotq_des_next - 2.*dotq + dotq_des) 
//             - 1./timestep/timestep*(q - q_des);
// }

void MagnetoTrajectoryManager::updateContact(int moving_foot_idx){
    // contact
    contact_list_.clear();
    for(auto it : foot_contact_map_){
        it.second->updateContactSpec();
        if(moving_foot_idx!=it.first)
            contact_list_.push_back(it.second);
    }

}