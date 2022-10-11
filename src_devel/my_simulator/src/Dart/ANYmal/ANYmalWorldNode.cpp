#include <../my_utils/Configuration.h>
#include <my_robot_core/anymal_core/anymal_interface.hpp>
#include <my_simulator/Dart/ANYmal/ANYmalWorldNode.hpp>

#include <my_utils/Math/MathUtilities.hpp>


ANYmalWorldNode::ANYmalWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;

    // ---- GET SKELETON
    robot_ = world_->getSkeleton("anymalc-ur3");
    ground_ = world_->getSkeleton("ground_skeleton");  

    R_ground_ = ground_->getBodyNode("ground_link")
                        ->getWorldTransform().linear();
    p_ground_ = ground_->getBodyNode("ground_link")
                        ->getWorldTransform().translation();  
    
    // ---- GET INFO FROM SKELETON
    // CheckRobotSkeleton(robot_);
    n_dof_ = robot_->getNumDofs();

    // ---- PLOT?
    b_plot_result_ = true;
   
    // ---- SET POSITION LIMIT
    // setPositionLimitEnforced

    // contact dinstance
    contact_threshold_ = 0.005;
    for(int i(0); i<ANYmal::n_leg; ++i){
        contact_distance_[i] = 0.05;         
    }
    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

    // ---- SET INTERFACE
    interface_ = new ANYmalInterface();
    sensor_data_ = new ANYmalSensorData();
    command_ = new ANYmalCommand();
}

ANYmalWorldNode::~ANYmalWorldNode() {
    delete interface_;
    delete sensor_data_;
    delete command_;
}


void ANYmalWorldNode::CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel){
    size_t n_bodynode = skel->getNumBodyNodes();
    std::string bn_name;

    // check inertia
    Eigen::MatrixXd I_tmp;
    bool b_check;
    for(size_t idx = 0; idx<n_bodynode; ++idx){
        bn_name = skel->getBodyNode(idx)->getName();
        // I_tmp = skel->getBodyNode(idx)->getInertia().getMoment();
        // my_utils::pretty_print(I_tmp, std::cout, bn_name);
        // b_check=skel->getBodyNode(idx)->getInertia().verify(true, 1e-5);
        I_tmp = skel->getBodyNode(idx)->getArticulatedInertia();
        my_utils::pretty_print(I_tmp, std::cout, bn_name);
    }    
}

void ANYmalWorldNode::customPostStep(){

}

void ANYmalWorldNode::enableButtonFlag(uint16_t key) {
    std::cout << "button(" << (char)key << ") pressed handled @ ANYmalWorldNode::enableButtonFlag" << std::endl;
    ((ANYmalInterface*)interface_) -> interrupt_ -> setFlags(key);    
}

void ANYmalWorldNode::customPreStep() {

    static Eigen::VectorXd qInit = robot_->getPositions();

    t_ = (double)count_ * servo_rate_;

    Eigen::VectorXd q = robot_->getPositions();
    Eigen::VectorXd qdot = robot_->getVelocities();

    for(int i=0; i< ANYmal::n_adof; ++i) {
        sensor_data_->q[i] = q[ANYmal::idx_adof[i]];
        sensor_data_->qdot[i] = qdot[ANYmal::idx_adof[i]];
        sensor_data_->tau_cmd_prev[i] = trq_cmd_[ANYmal::idx_adof[i]];
    }

    for(int i=0; i< ANYmal::n_vdof; ++i) {
        sensor_data_->virtual_q[i] = q[ANYmal::idx_vdof[i]];
        sensor_data_->virtual_qdot[i] = qdot[ANYmal::idx_vdof[i]];
    }    

    // update contact_distance_
    UpdateContactDistance_();
    // update sensor_data_->b_foot_contact 
    UpdateContactSwitchData_();
    // update sensor_data_->foot_wrench
    UpdateContactWrenchData_();

    // --------------------------------------------------------------
    //          COMPUTE COMMAND - desired joint acc/trq etc
    // --------------------------------------------------------------
    ((ANYmalInterface*)interface_)->getCommand(sensor_data_, command_);    

    // trq_cmd_.setZero();
    // for(int i=0; i< ANYmal::n_adof; ++i) {
    //     trq_cmd_[ANYmal::idx_adof[i]] =
    //                       command_->jtrq[i] + 
    //                       kd_[i] * (command_->qdot[i] - sensor_data_->qdot[i]) +
    //                       kp_[i] * (command_->q[i] - sensor_data_->q[i]);
    // }

    for(int i=0; i< ANYmal::n_adof; ++i) {
        trq_cmd_[ANYmal::idx_adof[i]] = 
                          kd_[i] * (command_->qdot[i] - sensor_data_->qdot[i]) +
                          kp_[i] * (command_->q[i] - sensor_data_->q[i]);
    }


    static int init_count = 0;   
    if(init_count++ < 50)
    {
        trq_cmd_.setZero();
    }

    EnforceTorqueLimit();
    setFrictionCoeff();
    robot_->setForces(trq_cmd_);

    // --------------------------------------------------------------
    //          Plot
    // --------------------------------------------------------------

    if (b_plot_result_) {
        if (((ANYmalInterface*)interface_)->IsPlannerUpdated()) {
            PlotResult_();
        }
        if (((ANYmalInterface*)interface_)->IsFootPlannerUpdated()) {
            PlotFootStepResult_();
        }
    }

    saveData();
    count_++;
}

void ANYmalWorldNode::saveData() {

    // joint command
    my_utils::saveVector(sensor_data_->qdot, "qdot_sen_simulation");
    my_utils::saveVector(command_->qdot, "qdot_cmd_simulation");
    my_utils::saveVector(sensor_data_->q, "q_sen_simulation");
    my_utils::saveVector(command_->q, "q_cmd_simulation");
    my_utils::saveVector(command_->jtrq, "jtrq_simulation");    
    my_utils::saveVector(trq_cmd_, "jtrq_cmd_simulation");    
    // my_utils::pretty_print(command_->jtrq, std::cout, "command_->jtrq");
    // my_utils::pretty_print(command_->qdot,std::cout, "command_->qdot");
    // my_utils::pretty_print(sensor_data_->qdot, std::cout, "sensor_data_->qdot");
    // my_utils::pretty_print(command_->q, std::cout, "command_->q");
    // my_utils::pretty_print(sensor_data_->q, std::cout, "sensor_data_->q"); 
    // std::cout<<"----------------"<<std::endl;
    // my_utils::pretty_print(trq_cmd_, std::cout, "trq_cmd_");
    // std::cout<<"----------------"<<std::endl;

    // contact force
    std::string filename;
    for(int ii(0); ii<ANYmal::n_leg;++ii){
        filename = ANYmalFoot::Names[ii] + "_wrench_local";  
        my_utils::saveVector(sensor_data_->foot_wrench[ii],filename);
    }

    // com position
    Eigen::Vector3d pcom = robot_->getCOM();
    my_utils::saveVector(pcom, "com_simulation");  

}

void ANYmalWorldNode::EnforceTorqueLimit()  {
    for(int i=0; i< n_dof_; ++i) {
        trq_cmd_[i] = trq_cmd_[i] >  trq_lb_[i] ? trq_cmd_[i] : trq_lb_[i];
        trq_cmd_[i] = trq_cmd_[i] <  trq_ub_[i] ? trq_cmd_[i] : trq_ub_[i];
    }    
}

void ANYmalWorldNode::setFrictionCoeff(){
    // =========================================================================
    // Friction & Restitution Coefficient
    // =========================================================================
    ground_->getBodyNode("ground_link")->setFrictionCoeff(1.0);

    robot_->getBodyNode(ANYmalFoot::LinkIdx[ANYmalFoot::RF])->setFrictionCoeff(coef_fric_[ANYmalFoot::RF]);
    robot_->getBodyNode(ANYmalFoot::LinkIdx[ANYmalFoot::LF])->setFrictionCoeff(coef_fric_[ANYmalFoot::LF]);
    robot_->getBodyNode(ANYmalFoot::LinkIdx[ANYmalFoot::LH])->setFrictionCoeff(coef_fric_[ANYmalFoot::LH]);
    robot_->getBodyNode(ANYmalFoot::LinkIdx[ANYmalFoot::RH])->setFrictionCoeff(coef_fric_[ANYmalFoot::RH]);
}


void ANYmalWorldNode::PlotResult_() {
    // world_->removeAllSimpleFrames();

    Eigen::VectorXd com_pos = Eigen::VectorXd::Zero(3);

    ((ANYmalInterface*)interface_)-> GetOptimalCoM(com_pos);
    Eigen::Isometry3d com_tf = Eigen::Isometry3d::Identity();
    com_tf.translation() = com_pos;
    std::vector <std::pair<double, Eigen::Vector3d>> feasible_com_list;
    ((ANYmalInterface*)interface_)-> GetFeasibleCoM(feasible_com_list); 

    // set color
    Eigen::Vector4d feasible_color = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0); // blue
    Eigen::Vector4d infeasible_color = Eigen::Vector4d(1.0, 0, 0.0, 1.0); // red

    // set Shape
    std::shared_ptr<dart::dynamics::PointCloudShape> feasible_shape =
        std::make_shared<dart::dynamics::PointCloudShape>(
            dart::dynamics::PointCloudShape(0.003));

    std::shared_ptr<dart::dynamics::PointCloudShape> infeasible_shape =
        std::make_shared<dart::dynamics::PointCloudShape>(
            dart::dynamics::PointCloudShape(0.003));

    std::shared_ptr<dart::dynamics::SphereShape> s_shape =
        std::make_shared<dart::dynamics::SphereShape>(
            dart::dynamics::SphereShape(0.01));


    for (auto it = feasible_com_list.begin(); it != feasible_com_list.end(); it++) {
        if( (*it).first > 0.0 )
            feasible_shape->addPoint((*it).second);
        else
            infeasible_shape->addPoint((*it).second);
    }
    std::cout<<"PlotResult :Feasible Points:" << feasible_shape->getNumPoints()
     << ", Infeasible Points:" << infeasible_shape->getNumPoints() << std::endl;

    // set Frame
    dart::dynamics::SimpleFramePtr feasible_com_frame, infeasible_com_frame;
    dart::dynamics::SimpleFramePtr optimal_com_frame;
    
    feasible_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com_feasible");
    infeasible_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com_infeasible");
    optimal_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com_target", com_tf);
    
    feasible_com_frame->setShape(feasible_shape);
    feasible_com_frame->getVisualAspect(true)->setColor(feasible_color);
    world_->addSimpleFrame(feasible_com_frame);

    infeasible_com_frame->setShape(infeasible_shape);
    infeasible_com_frame->getVisualAspect(true)->setColor(infeasible_color);
    world_->addSimpleFrame(infeasible_com_frame);

    optimal_com_frame->setShape(s_shape);
    optimal_com_frame->getVisualAspect(true)->setColor(infeasible_color);
    world_->addSimpleFrame(optimal_com_frame);

    
}

void ANYmalWorldNode::PlotForce_(int fidx, const Eigen::Vector3d& frc_foot){

    static int plotFrcCnt[ANYmal::n_leg] = {0};
    if(plotFrcCnt[fidx]++ > 100){
        plotFrcCnt[fidx] = 0;   

        // set color
        Eigen::Isometry3d foot_tf = robot_->getBodyNode(ANYmalFoot::LinkIdx[fidx])->getTransform();        
        Eigen::Vector3d arrow_tail = Eigen::Vector3d::Zero(); // foot_tf.translation();
        Eigen::Vector3d arrow_head(1.0, 1.0, 1.0);// arrow_tail + frc_foot;
        
        // // set shape
        // dart::dynamics::ArrowShapePtr arrow_shape = 
        //     std::make_shared<dart::dynamics::ArrowShape>(
        //         dart::dynamics::ArrowShape(arrow_tail, 
        //             arrow_head,
        //             dart::dynamics::ArrowShape::Properties(0.002, 1.8),
        //             dart::Color::Orange(1.0)) );

        // // set frame     
        // dart::dynamics::SimpleFramePtr frc_frame 
        // = std::make_shared<dart::dynamics::SimpleFrame>(
        //     dart::dynamics::Frame::World(), "frc_frame");

        // if( frc_frame->getShape()== arrow_shape){
        //     frc_frame->removeVisualAspect();
        // }
        // frc_frame->setShape(arrow_shape);
        // world_->addSimpleFrame(frc_frame);


        std::cout<< robot_->getBodyNode(ANYmalFoot::LinkIdx[fidx])->getName() <<"= (";
        std::cout<< frc_foot.transpose() <<"), " <<frc_foot.norm();
        Eigen::Vector3d zdir = foot_tf.linear().col(2);
        std::cout<< ", zdir = ("<< zdir.transpose() <<")" << std::endl;
    }
}


void ANYmalWorldNode::PlotFootStepResult_() {
    // world_->removeAllSimpleFrames();
    
    Eigen::VectorXd foot_pos;
    ((ANYmalInterface*)interface_)-> GetNextFootStep(foot_pos);
    Eigen::Isometry3d foot_tf = robot_->getBodyNode("base")->getTransform();
    foot_tf.translation() = foot_pos;  
    // my_utils::pretty_print(foot_pos, std::cout, "PlotFootStepResult_"); 

    // set shape
    dart::dynamics::BoxShapePtr foot_shape =
        std::make_shared<dart::dynamics::BoxShape>(
            dart::dynamics::BoxShape(Eigen::Vector3d(0.02, 0.02, 0.02))); // Eigen::Vector3d(0.02, 0.02, 0.001)

    // set color
    Eigen::Vector4d foot_step_color = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0); // blue

    // set frame
    dart::dynamics::SimpleFramePtr foot_frame;
    foot_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "next_foot", foot_tf);
    foot_frame->setShape(foot_shape);
    foot_frame->getVisualAspect(true)->setColor(foot_step_color);
    world_->addSimpleFrame(foot_frame);   
}

void ANYmalWorldNode::setParameters(const YAML::Node& simulation_cfg) {
    // will be called @ Main.cpp, after created
    try {
        my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "torque_limit", torque_limit_);
        my_utils::readParameter(simulation_cfg, "plot_result", b_plot_result_);
        
        // setting 
        my_utils::readParameter(simulation_cfg["contact_params"], "friction", coef_fric_);

    } 
    catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    // ---- SET TORQUE LIMIT
    trq_lb_ = Eigen::VectorXd::Constant(n_dof_, -torque_limit_);
    trq_ub_ = Eigen::VectorXd::Constant(n_dof_, torque_limit_);
    // trq_lb_ = robot_->getForceLowerLimits();
    // trq_ub_ = robot_->getForceUpperLimits();

    // ---- CHECK 
    // std::cout<<"trq_lb_ = "<<trq_lb_.transpose() << std::endl;
    // std::cout<<"trq_ub_ = "<<trq_ub_.transpose() << std::endl; 
    // my_utils::pretty_print(coef_fric_, std::cout, "sim : coef_fric_");

    setFrictionCoeff();
}

void ANYmalWorldNode::UpdateContactDistance_() {
    // get normal distance from the ground link frame R_ground
    Eigen::MatrixXd R_gw = R_ground_.transpose();
    Eigen::MatrixXd p_gw = - R_gw * p_ground_;   
    Eigen::Vector3d dist;    
    for(int i(0); i<ANYmal::n_leg; ++i){
        dist = p_gw + R_gw*robot_->getBodyNode(ANYmalFoot::LinkIdx[i]) // COP frame node?
                                    ->getWorldTransform().translation();
        contact_distance_[i] = std::max(0.0, dist[2]);
        // std::cout << contact_distance_[i] << ", ";
    }
    // std::cout << std::endl;
}


void ANYmalWorldNode::UpdateContactSwitchData_() {
    
    // TODO : distance base -> force base ?  
    for(int ii(0); ii<ANYmal::n_leg;++ii)
        sensor_data_->b_foot_contact[ii]
            = contact_distance_[ii] < contact_threshold_;


    // static bool first_contact = false;
    // if(!first_contact) {
    //     if(alfoot_contact || blfoot_contact || arfoot_contact || brfoot_contact) {
    //         Eigen::Vector3d p_com = robot_->getCOM();    
    //         std::cout<< count_<< " / first_contact! com position :" << p_com(0) << "," << p_com(1) << "," << p_com(2) << std::endl;

    //         Eigen::Vector3d p_base = robot_->getBodyNode("base")->getTransform().translation();    
    //         std::cout<< "first_contact! p_base position :" << p_base(0) << "," << p_base(1) << "," << p_base(2) << std::endl;

    //         // std::cout<< "wrench" << sensor_data_->alf_wrench(2) << ","  << sensor_data_->arf_wrench(2) << ","  
    //         //                         << sensor_data_->blf_wrench(2) << "," << sensor_data_->brf_wrench(2) << std::endl;
    //         std::cout<< "-------------------    first_contact   ---------------------" << std::endl;
    //         first_contact = true;
    //         // exit(0);
    //     }
    // }

}


void ANYmalWorldNode::UpdateContactWrenchData_() {

    // (contact wrench)
    std::array<Eigen::VectorXd, ANYmal::n_leg> wrench_local_list;
    std::array<Eigen::VectorXd, ANYmal::n_leg> wrench_global_list;
    for(int ii=0; ii<ANYmal::n_leg; ++ii) {
        wrench_local_list[ii]=(Eigen::VectorXd::Zero(6));
        wrench_global_list[ii]=(Eigen::VectorXd::Zero(6));
    }

    const dart::collision::CollisionResult& _result =
                            world_->getLastCollisionResult();

    
    for(int ii=0; ii<ANYmal::n_leg; ++ii){
        int contact_link_idx= ANYmalFoot::LinkIdx[ii];
        for (const auto& contact : _result.getContacts()) 
        {
            for (const auto& shapeNode :
                    robot_->getBodyNode(contact_link_idx)->
                    getShapeNodesWith<dart::dynamics::CollisionAspect>()) 
            {
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();

                if ( shapeNode == contact.collisionObject1->getShapeFrame() )               
                    w_c.tail(3) = contact.force;                    
                else if( shapeNode == contact.collisionObject2->getShapeFrame())
                    w_c.tail(3) = - contact.force;                
                else { continue; }
                
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    robot_->getBodyNode(contact_link_idx) // Foot COP Frame
                        ->getTransform(dart::dynamics::Frame::World());
                        
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::Isometry3d T_cw = T_wc.inverse();

                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::MatrixXd AdT_cw = dart::math::getAdTMatrix(T_cw);

                Eigen::VectorXd w_a = AdT_ca.transpose() * w_c;
                Eigen::VectorXd w_w = AdT_cw.transpose() * w_c;
                
                wrench_local_list[ii] += w_a;
                wrench_global_list[ii] += w_w;  

            }  
        }
        // my_utils::pretty_print(wrench_global_list[ii],std::cout,"wrench_global_list");
    }

    // sensor_data_->foot_wrench = wrench_local_list;  
    sensor_data_->foot_wrench = wrench_global_list;


}
