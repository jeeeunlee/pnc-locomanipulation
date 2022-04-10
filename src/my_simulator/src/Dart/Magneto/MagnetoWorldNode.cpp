#include <../my_utils/Configuration.h>
#include <my_robot_core/magneto_core/magneto_interface.hpp>
#include <my_simulator/Dart/Magneto/MagnetoWorldNode.hpp>

#include <my_utils/Math/MathUtilities.hpp>


MagnetoWorldNode::MagnetoWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;

    // ---- GET SKELETON
    robot_ = world_->getSkeleton("magneto");
    ground_ = world_->getSkeleton("ground_skeleton");   
    R_ground_ = ground_->getBodyNode("ground_link")
                        ->getWorldTransform().linear();
    p_ground_ = ground_->getBodyNode("ground_link")
                        ->getWorldTransform().translation();    
    
    // ---- GET INFO FROM SKELETON
    // CheckRobotSkeleton(robot_);
    n_dof_ = robot_->getNumDofs();   
    std::cout<< "Magneto mass : " <<  robot_->getMass() << std::endl;     

    // ---- PLOT?
    b_plot_result_ = true;
   
    // ---- SET POSITION LIMIT
    // setPositionLimitEnforced

    // contact dinstance
    contact_threshold_ = 0.005;
    for(int i(0); i<Magneto::n_leg; ++i){
        contact_distance_[i] = 0.05; 
        surface_normal_[i] = Eigen::Vector3d::Zero();
    }


    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

    // ---- SET INTERFACE
    interface_ = new MagnetoInterface();
    sensor_data_ = new MagnetoSensorData();
    command_ = new MagnetoCommand();

    // ---- SET control parameters %% motion script
    run_mode_ = ((MagnetoInterface*)interface_)->getRunMode();
}

MagnetoWorldNode::~MagnetoWorldNode() {
    delete interface_;
    delete sensor_data_;
    delete command_;
}


void MagnetoWorldNode::CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel){
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

void MagnetoWorldNode::customPostStep(){

}

void MagnetoWorldNode::enableButtonFlag(uint16_t key) {
    std::cout << "button(" << (char)key << ") pressed handled @ MagnetoWorldNode::enableButtonFlag" << std::endl;
    ((MagnetoInterface*)interface_) -> interrupt_ -> setFlags(key);    
}

void MagnetoWorldNode::customPreStep() {

    static Eigen::VectorXd qInit = robot_->getPositions();

    t_ = (double)count_ * servo_rate_;

    Eigen::VectorXd q = robot_->getPositions();
    Eigen::VectorXd qdot = robot_->getVelocities();

    for(int i=0; i< Magneto::n_adof; ++i) {
        sensor_data_->q[i] = q[Magneto::idx_adof[i]];
        sensor_data_->qdot[i] = qdot[Magneto::idx_adof[i]];
        sensor_data_->tau_cmd_prev[i] = trq_cmd_[Magneto::idx_adof[i]];
    }

    for(int i=0; i< Magneto::n_vdof; ++i) {
        sensor_data_->virtual_q[i] = q[Magneto::idx_vdof[i]];
        sensor_data_->virtual_qdot[i] = qdot[Magneto::idx_vdof[i]];
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
    ((MagnetoInterface*)interface_)->getCommand(sensor_data_, command_);    

    trq_cmd_.setZero();
    for(int i=0; i< Magneto::n_adof; ++i) {
        trq_cmd_[Magneto::idx_adof[i]] 
                        = command_->jtrq[i] + 
                          kd_ * (command_->qdot[i] - sensor_data_->qdot[i]) +
                          kp_ * (command_->q[i] - sensor_data_->q[i]);
    }
    // spring in gimbal    
    // double ks = 1.0;// N/rad
    // for(int i=6; i< Magneto::n_vdof; ++i) {
    //     trq_cmd_[Magneto::idx_vdof[i]] = ks * ( 0.0 - sensor_data_->virtual_q[i]);
    // }

    EnforceTorqueLimit();
    updateContactEnvSetup();
    setFrictionCoeff();
    ApplyMagneticForce();
    robot_->setForces(trq_cmd_);

    // --------------------------------------------------------------
    //          Plot
    // --------------------------------------------------------------

    if (b_plot_result_) {
        if (((MagnetoInterface*)interface_)->IsPlannerUpdated()) {
            PlotResult_();
        }
        if (((MagnetoInterface*)interface_)->IsFootPlannerUpdated()) {
            PlotFootStepResult_();
        }
    }

    saveData();
    count_++;
}

void MagnetoWorldNode::saveData() {
    // mag onoff
    double b_mag = 0.0;
    std::string filename;
    int link_idx;
    for(int i(0); i<Magneto::n_leg; ++i){
        filename = MagnetoFoot::Names[i] + "_mag_onoff_simulation";
        b_mag = (double) command_->magnetism_onoff[i];
        my_utils::saveValue(b_mag, filename);
    }

    // joint command
    my_utils::saveVector(sensor_data_->qdot, "qdot_sen_simulation");
    my_utils::saveVector(command_->qdot, "qdot_cmd_simulation");
    my_utils::saveVector(sensor_data_->q, "q_sen_simulation");
    my_utils::saveVector(command_->q, "q_cmd_simulation");
    my_utils::saveVector(command_->jtrq, "jtrq_simulation");    
    // my_utils::pretty_print(command_->jtrq, std::cout, "command_->jtrq");
    // my_utils::pretty_print(command_->qdot,std::cout, "command_->qdot");
    // my_utils::pretty_print(sensor_data_->qdot, std::cout, "sensor_data_->qdot");
    // my_utils::pretty_print(command_->q, std::cout, "command_->q");
    // my_utils::pretty_print(sensor_data_->q, std::cout, "sensor_data_->q"); 

    // contact force
    my_utils::saveVector(sensor_data_->alf_wrench, "alf_wrench_local");
    my_utils::saveVector(sensor_data_->blf_wrench, "blf_wrench_local");   
    my_utils::saveVector(sensor_data_->arf_wrench, "arf_wrench_local");   
    my_utils::saveVector(sensor_data_->brf_wrench, "brf_wrench_local");   



}

void MagnetoWorldNode::EnforceTorqueLimit()  {
    for(int i=0; i< n_dof_; ++i) {
        trq_cmd_[i] = trq_cmd_[i] >  trq_lb_[i] ? trq_cmd_[i] : trq_lb_[i];
        trq_cmd_[i] = trq_cmd_[i] <  trq_ub_[i] ? trq_cmd_[i] : trq_ub_[i];
    }    
}

void MagnetoWorldNode::setFrictionCoeff(){
    // =========================================================================
    // Friction & Restitution Coefficient
    // =========================================================================
    ground_->getBodyNode("ground_link")->setFrictionCoeff(0.7);

    robot_->getBodyNode("BL_foot_link")->setFrictionCoeff(coef_fric_[MagnetoFoot::BL]);
    robot_->getBodyNode("AL_foot_link")->setFrictionCoeff(coef_fric_[MagnetoFoot::AL]);
    robot_->getBodyNode("AR_foot_link")->setFrictionCoeff(coef_fric_[MagnetoFoot::AR]);
    robot_->getBodyNode("BR_foot_link")->setFrictionCoeff(coef_fric_[MagnetoFoot::BR]);

    robot_->getBodyNode("BL_foot_link_3")->setFrictionCoeff(coef_fric_[MagnetoFoot::BL]);
    robot_->getBodyNode("AL_foot_link_3")->setFrictionCoeff(coef_fric_[MagnetoFoot::AL]);
    robot_->getBodyNode("AR_foot_link_3")->setFrictionCoeff(coef_fric_[MagnetoFoot::AR]);
    robot_->getBodyNode("BR_foot_link_3")->setFrictionCoeff(coef_fric_[MagnetoFoot::BR]);
}

void MagnetoWorldNode::updateContactEnvSetup() {
    int foot_idx =  ((MagnetoInterface*)interface_)->getCurrentMovingFootIdx();
    if(foot_idx > 0 && foot_idx<Magneto::n_leg)
        ((MagnetoInterface*)interface_)->getSimulationEnvironment(
                                        coef_fric_[foot_idx], 
                                        magnetic_force_[foot_idx]);
} 

void MagnetoWorldNode::ApplyMagneticForce()  {
    bool is_force_local = false; 
    double fm(0.);
    Eigen::Vector3d force = Eigen::Vector3d::Zero();   
    Eigen::Vector3d location = Eigen::Vector3d::Zero();  
    

    for(int i(0); i<Magneto::n_leg; ++i) {
        // command_->magnetism_onoff : [mag_onoff]
        if( command_->magnetism_onoff[i] ) {
            fm = - magnetic_force_[i];
        } else {
            // distance 0->1 , infinite->0            
            double distance_constant = contact_threshold_ * 4.; // 0.045
            double res_fm_ratio = (distance_constant / (contact_distance_[i] + distance_constant));
            res_fm_ratio = res_fm_ratio * res_fm_ratio * (residual_magnetism_[i]/100.); 
            fm = - res_fm_ratio * magnetic_force_[i];
            // std::cout<<"res: dist = "<<contact_distance_[i]<<", res_fm_ratio=" << res_fm_ratio << std::endl;
        }
        if(is_force_local){
            force = Eigen::Vector3d::Zero();
            force[2] = fm;        
            robot_->getBodyNode(MagnetoFoot::LinkIdx[i])->addExtForce(force, location, true);
        }else{
            force = fm*surface_normal_[i];
            robot_->getBodyNode(MagnetoFoot::LinkIdx[i])->addExtForce(force, location, false);            
            // std::cout<< MagnetoFoot::Names[i] <<"="<< force.transpose() << std::endl;
            PlotForce_(i, force);                     
        }
    }
}

void MagnetoWorldNode::PlotResult_() {
    // world_->removeAllSimpleFrames();

    Eigen::VectorXd com_pos = Eigen::VectorXd::Zero(3);

    ((MagnetoInterface*)interface_)-> GetOptimalCoM(com_pos);
    Eigen::Isometry3d com_tf = Eigen::Isometry3d::Identity();
    com_tf.translation() = com_pos;
    std::vector <std::pair<double, Eigen::Vector3d>> feasible_com_list;
    ((MagnetoInterface*)interface_)-> GetFeasibleCoM(feasible_com_list); 

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

void MagnetoWorldNode::PlotForce_(int fidx, const Eigen::Vector3d& frc_foot){
    static int plotFrcCnt = 0;
    if(plotFrcCnt++ > 10000){
        plotFrcCnt = 0;   
        // magnetic force
        // set color
        Eigen::Isometry3d foot_tf = robot_->getBodyNode(MagnetoFoot::LinkIdx[fidx])->getTransform();
        Eigen::Vector3d arrow_tail = Eigen::Vector3d::Zero(); // foot_tf.translation();
        Eigen::Vector3d arrow_head(1.0, 1.0, 1.0);// arrow_tail + frc_foot;


        // set shape
        dart::dynamics::ArrowShapePtr arrow_shape = 
            std::make_shared<dart::dynamics::ArrowShape>(
                dart::dynamics::ArrowShape(arrow_tail, 
                    arrow_head,
                    dart::dynamics::ArrowShape::Properties(0.002, 1.8),
                    dart::Color::Orange(1.0)) );

        // set frame     
        dart::dynamics::SimpleFramePtr frc_frame 
        = std::make_shared<dart::dynamics::SimpleFrame>(
            dart::dynamics::Frame::World(), "frc_frame");

        if( frc_frame->getShape()== arrow_shape){
            frc_frame->removeVisualAspect();
        }
        frc_frame->setShape(arrow_shape);
        world_->addSimpleFrame(frc_frame);
    }
}


void MagnetoWorldNode::PlotFootStepResult_() {
    // world_->removeAllSimpleFrames();
    
    Eigen::VectorXd foot_pos;
    ((MagnetoInterface*)interface_)-> GetNextFootStep(foot_pos);
    Eigen::Isometry3d foot_tf = robot_->getBodyNode("base_link")->getTransform();
    foot_tf.translation() = foot_pos;  
    my_utils::pretty_print(foot_pos, std::cout, "PlotFootStepResult_"); 

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

    // for mpc test

    std::cout<<"================================"<<std::endl;
    Eigen::Vector3d base_pos = robot_->getCOM();
    my_utils::pretty_print(base_pos, std::cout, "P_com");  

    std::string varname;
    Eigen::Isometry3d foot_iso;
    Eigen::VectorXd foot_pos_;
    Eigen::MatrixXd foot_rot_;
    Eigen::Quaterniond foot_quat_;
    for(int i(0); i<Magneto::n_leg; ++i){
        varname = MagnetoFoot::Names[i]+"config";
        foot_iso = robot_->getBodyNode(MagnetoFoot::LinkIdx[i])->getWorldTransform();
        foot_pos_ = foot_iso.translation();
        foot_rot_ = foot_iso.linear();
        foot_quat_ = Eigen::Quaternion<double>(foot_iso.linear());
        my_utils::pretty_print(foot_pos_, std::cout, varname);
        // my_utils::pretty_print(foot_rot_, std::cout, varname);
        my_utils::pretty_print(foot_quat_, std::cout, varname);
    }
    std::cout<<"================================"<<std::endl;
    

}

void MagnetoWorldNode::setParameters(const YAML::Node& simulation_cfg) {
    // will be called @ Main.cpp, after created
    try {
        my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "torque_limit", torque_limit_);
        my_utils::readParameter(simulation_cfg, "plot_result", b_plot_result_);
        
        // setting magnetic force
        my_utils::readParameter(simulation_cfg["contact_params"], "friction", coef_fric_);
        my_utils::readParameter(simulation_cfg["magnetism_params"], "magnetic_force", magnetic_force_);  
        my_utils::readParameter(simulation_cfg["magnetism_params"], "residual_magnetism", residual_magnetism_); 
        my_utils::readParameter(simulation_cfg, "contact_frame", contact_frame_type_);

        for(int i(0); i<Magneto::n_leg; ++i){
            if(contact_frame_type_==2)
                    my_utils::readParameter(simulation_cfg["contact_surface_noraml"], 
                                            MagnetoFoot::Names[i], surface_normal_[i]);                
            else surface_normal_[i] = R_ground_.col(2); // rz direction of ground link             
            
            if(surface_normal_[i].norm() > 1e-5) surface_normal_[i] /= surface_normal_[i].norm(); 
        }
                

        // read motion scripts
        std::string motion_file_name;      
        my_utils::readParameter(simulation_cfg, "motion_script", motion_file_name);
        ReadMotions_(motion_file_name);
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
    // my_utils::pretty_print(magnetic_force_, std::cout, "sim : magnetic_force_");
    // my_utils::pretty_print(residual_magnetism_, std::cout, "sim : residual_magnetism_");
    // my_utils::pretty_print(coef_fric_, std::cout, "sim : coef_fric_");

    setFrictionCoeff();
}

void MagnetoWorldNode::ReadMotions_(const std::string& _motion_file_name) {
    std::ostringstream motion_file_name;    
    motion_file_name << THIS_COM << _motion_file_name;
    
    try { 
        YAML::Node motion_cfg = YAML::LoadFile(motion_file_name.str());
        int num_motion;
        int type_motion;
        my_utils::readParameter(motion_cfg, "num_motion", num_motion);
        my_utils::readParameter(motion_cfg, "type_motion", type_motion);
        for(int i(0); i<num_motion; ++i){
            std::ostringstream stringStream;
            stringStream << "motion" << i;
            std::string conf = stringStream.str();
            switch(type_motion){
                case 0: //clmibing
                    ((MagnetoInterface*)interface_)->
                    AddScriptMotion(motion_cfg[conf]);
                    break;
                case 1: //walking
                case 2: //balancing
                    ((MagnetoInterface*)interface_)->
                    AddScriptMotion(motion_cfg[conf]);
                    break;
            }
            
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

void MagnetoWorldNode::UpdateContactDistance_() {
    // get normal distance from the ground link frame R_ground

    Eigen::Vector3d dist;
    double d;
    // std::cout <<" contact_distance_  = ";
    for(int i(0); i<Magneto::n_leg; ++i){
        dist = - p_ground_ + robot_->getBodyNode(MagnetoFoot::LinkIdx[i]) // COP frame node?
                                    ->getWorldTransform().translation();
        d = dist.dot(surface_normal_[i]);
        contact_distance_[i] = std::max(0.0, d);
        // std::cout << contact_distance_[i] << ", ";
    }
    // std::cout << std::endl;
}


void MagnetoWorldNode::UpdateContactSwitchData_() {
    
    // TODO : distance base -> force base ?  
    sensor_data_->alfoot_contact 
        = contact_distance_[MagnetoFoot::AL] < contact_threshold_;
    sensor_data_->blfoot_contact
        = contact_distance_[MagnetoFoot::BL] < contact_threshold_;    
    sensor_data_->arfoot_contact
        = contact_distance_[MagnetoFoot::AR] < contact_threshold_;
    sensor_data_->brfoot_contact
        = contact_distance_[MagnetoFoot::BR] < contact_threshold_;

    // static bool first_contact = false;
    // if(!first_contact) {
    //     if(alfoot_contact || blfoot_contact || arfoot_contact || brfoot_contact) {
    //         Eigen::Vector3d p_com = robot_->getCOM();    
    //         std::cout<< count_<< " / first_contact! com position :" << p_com(0) << "," << p_com(1) << "," << p_com(2) << std::endl;

    //         Eigen::Vector3d p_base_link = robot_->getBodyNode("base_link")->getTransform().translation();    
    //         std::cout<< "first_contact! p_base_link position :" << p_base_link(0) << "," << p_base_link(1) << "," << p_base_link(2) << std::endl;

    //         // std::cout<< "wrench" << sensor_data_->alf_wrench(2) << ","  << sensor_data_->arf_wrench(2) << ","  
    //         //                         << sensor_data_->blf_wrench(2) << "," << sensor_data_->brf_wrench(2) << std::endl;
    //         std::cout<< "-------------------    first_contact   ---------------------" << std::endl;
    //         first_contact = true;
    //         // exit(0);
    //     }
    // }

}


void MagnetoWorldNode::UpdateContactWrenchData_() {

    // (contact COP link name, contact link node)
    std::vector<std::pair<std::string,dart::dynamics::BodyNode*>> bn_list;

    bn_list.clear();
    bn_list.push_back(std::make_pair("AL_foot_link",robot_->getBodyNode("AL_foot_link_3")));
    bn_list.push_back(std::make_pair("BL_foot_link",robot_->getBodyNode("BL_foot_link_3")));
    bn_list.push_back(std::make_pair("AR_foot_link",robot_->getBodyNode("AR_foot_link_3")));
    bn_list.push_back(std::make_pair("BR_foot_link",robot_->getBodyNode("BR_foot_link_3")));

    // (contact wrench)
    std::vector<Eigen::Vector6d> wrench_local_list;
    std::vector<Eigen::Vector6d> wrench_global_list;  

    wrench_local_list.clear();    
    wrench_global_list.clear();
    for(int i=0; i<bn_list.size(); ++i) {
        wrench_local_list.push_back(Eigen::VectorXd::Zero(6));
        wrench_global_list.push_back(Eigen::VectorXd::Zero(6));
    }

    const dart::collision::CollisionResult& _result =
                            world_->getLastCollisionResult();

    for (const auto& contact : _result.getContacts()) 
    {
        for(int i=0; i<bn_list.size(); ++i) 
        {
            for (const auto& shapeNode :
                bn_list[i].second->getShapeNodesWith<dart::dynamics::CollisionAspect>()) 
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
                    robot_->getBodyNode(bn_list[i].first) // Foot COP Frame
                        ->getTransform(dart::dynamics::Frame::World());
                        
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::Isometry3d T_cw = T_wc.inverse();

                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::MatrixXd AdT_cw = dart::math::getAdTMatrix(T_cw);

                Eigen::VectorXd w_a = AdT_ca.transpose() * w_c;
                Eigen::VectorXd w_w = AdT_cw.transpose() * w_c;
                
                wrench_local_list[i] += w_a;
                wrench_global_list[i] += w_w;  

            }  
        }      
    }

    // std::cout<<"-------------------------------" << std::endl;
   
    // for(int i=0; i<wrench_local_list.size(); ++i)
    // {
    //     std::string printname = bn_list[i].first;
    //     // std::ostream printname;
    //     // printname << bn_list[i].first;
    //     Eigen::VectorXd wrench = wrench_local_list[i];
    //     my_utils::pretty_print(wrench, std::cout, "wrench");
    //     wrench = wrench_global_list[i];
    //     my_utils::pretty_print(wrench, std::cout, "wrench_w");
    // }
    // std::cout<<"-------------------------------" << std::endl;

    sensor_data_->alf_wrench = wrench_local_list[0];
    sensor_data_->blf_wrench = wrench_local_list[1];
    sensor_data_->arf_wrench = wrench_local_list[2];
    sensor_data_->brf_wrench = wrench_local_list[3];
    
}
