// #include <my_pnc/MagnetoPnC/ContactSet/ContactSet.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoGIWCPlanner.hpp>
#include <my_geometry/Polytope/Polytope.h>
#include <my_utils/IO/IOUtilities.hpp>
#include "my_utils/Math/pseudo_inverse.hpp"
#include <random>


MagnetoGIWCPlanner::MagnetoGIWCPlanner(RobotSystem* robot) {
    my_utils::pretty_constructor(1, "Magneto GIWC Planner");

    robot_ = robot;
    // *robot_temp_ = *robot; todo later -> copy constructor
    robot_temp_ = new RobotSystem( 6+3*4, robot_->getFileName() );   
 
    robot_->getActuatedJointIdx(adof_idx_);
    _buildCoMJacobian();
    fm_over_m_ = 0.0;
}

void MagnetoGIWCPlanner::setContact(const std::vector<ContactSpec*> & contact_list, ContactSpecType contact_type) {
    setContact(contact_list, contact_list, contact_type);
}

void MagnetoGIWCPlanner::setContact(const std::vector<ContactSpec*> & contact_list, 
                                const std::vector<ContactSpec*> & real_contact_list, 
                                ContactSpecType contact_type) {
    contact_type_ = contact_type;

    n_contact_ = contact_list.size();
    if(n_contact_ < 1)
    {
        std::cout<<"empty contact_list in the MagnetoGIWCPlanner" << std::endl;
        exit(0);
    }
    // std::cout<< "MotionPlanner Contact num = " << contact_list.size() << std::endl; 
    
    Eigen::MatrixXd Uf_i;
    int dim_rows, dim_cols, dim_rows_i, dim_cols_i;

    
    contact_list[0]->getRFConstraintMtx(Uf_);
    _ineqMat2Uf(Uf_);
    // my_utils::pretty_print(Uf_, std::cout, "Uf_i");
    dim_rows = Uf_.rows();
    dim_cols = Uf_.cols();


    link_idx_.clear();
    int link_idx = ((BodyFramePointContactSpec*)contact_list[0])->getLinkIdx();
    link_idx_.push_back(link_idx);
    
    // std::cout << "size of Uf_i : " << dim_rows << "," << dim_cols << std::endl;

    int dim_new_rf, dim_new_rf_cstr;
    for(int i(1); i<n_contact_; ++i) {

        link_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
        link_idx_.push_back(link_idx);

        contact_list[i]->getRFConstraintMtx(Uf_i);
        _ineqMat2Uf(Uf_i);
        dim_rows_i = Uf_i.rows();
        dim_cols_i = Uf_i.cols();

        Uf_.conservativeResize(dim_rows + dim_rows_i, dim_cols + dim_cols_i);
        Uf_.block(0, dim_cols, dim_rows, dim_cols_i).setZero();
        Uf_.block(dim_rows, 0, dim_rows_i, dim_cols).setZero();
        Uf_.block(dim_rows, dim_cols, dim_rows_i, dim_cols_i) = Uf_i;

        dim_rows += dim_rows_i;
        dim_cols += dim_cols_i;
    }

    _buildAstanceMatrix(contact_list);
    _buildContactPlane(contact_list);
    _buildContactJacobianNullspace(real_contact_list);
    _updateConvexHull();
}

void MagnetoGIWCPlanner::getFeasibleCOM(
    std::vector <std::pair<double, Eigen::Vector3d>>& _feasible_com_list) {
    _feasible_com_list = Feasible_com_list_;
}

bool MagnetoGIWCPlanner::findOptimalCOM(const Eigen::Vector3d& com_pos_ini, Eigen::Vector3d& com_pos_des) {

    // 0. Initialize
    Feasible_com_list_.clear();
    Eigen::Vector3d pos_opt = com_pos_ini;

    // 1. build abc, d (HyperPlance inequality for pos)
    Eigen::MatrixXd abc; 
    Eigen::VectorXd d;
    _buildHyperPlane(abc, d);
  
    // 2. find pos_opt maximizing minimum distance from the set of hyperplane (abc,d)
    Eigen::Vector3d pos_tmp;
    double maxmind(0.0), mind;
    int i(0), iter(1000);
    while(++i<iter)
    {
        // Eigen::Vector3d pos_disturbance = Eigen::Vector3d::Random();
        _getRandomSample(com_pos_ini, pos_tmp);
        mind = _getMinDistance(pos_tmp, abc, d);
        // std::cout<<"find com : mind = "<< mind << " at pd=" << pos_disturbance(0)<<", "<< pos_disturbance(1)<<", "<< pos_disturbance(2)<<std::endl;

        if(maxmind < mind)
        {
            maxmind = mind;
            pos_opt = pos_tmp;
        }

        Feasible_com_list_.push_back(std::make_pair(mind,pos_tmp));
    }
    com_pos_des = pos_opt;
    return (maxmind > 0); // return true if 
}

// protected ---------------------------------------------------------------------------------------
void MagnetoGIWCPlanner::_enforceJointLimit(const Eigen::VectorXd& q_curr, Eigen::VectorXd& dq) {
    Eigen::VectorXd q_lower = robot_->GetPositionLowerLimits();
    Eigen::VectorXd q_upper = robot_->GetPositionUpperLimits();
    Eigen::VectorXd dq_temp = dq;
    double q_limit_margin = 0.05;

    // 
    for(int i=6; i<n_dof_; i++) {
        dq[i] = (q_upper[i]-q_curr[i]) < dq[i] ? (q_upper[i]-q_curr[i]-q_limit_margin) : dq[i];
        dq[i] = (q_lower[i]-q_curr[i]) > dq[i] ? (q_lower[i]-q_curr[i]+q_limit_margin) : dq[i];        
    }

    // if( (dq_temp - dq).norm() > 1e-3 )
    // {
    //     my_utils::pretty_print(q_upper, std::cout, "q_upper");
    //     my_utils::pretty_print(q_curr, std::cout, "q_curr");
    //     my_utils::pretty_print(q_lower, std::cout, "q_lower");
    //     my_utils::pretty_print(dq, std::cout, "dq");
    //     my_utils::pretty_print(dq_temp, std::cout, "dq_temp");
    //     std::cout << "dq enforced" << std::endl;
    // }
}

void MagnetoGIWCPlanner::_getRandomSample(const Eigen::Vector3d& com_pos_ini, 
                                      Eigen::Vector3d& com_pos_sample)
{
    // double b_r(0.05);
    // com_pos_sample = com_pos_ini_ + b_r*Eigen::Vector3d::Random();
    
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(n_dof_);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.05, 0.05); // 0.174rad=10deg
    // for(int i=0; i<adof_idx_.size(); ++i)
    // {
    //     dq(adof_idx_[i]) = dis(gen);
    // }

    for(int i=0; i<n_dof_; ++i)
    {
        dq(i) = dis(gen);
    }

    // kinematic constraint 
    Eigen::VectorXd q_curr = robot_->getQ();
    dq = Nc_ * dq; // rigid contact condition 
    _enforceJointLimit(q_curr,dq);
    Eigen::VectorXd q = q_curr + dq; 
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(q.size());
    robot_temp_->updateSystem(q, qdot, false);    

    Eigen::MatrixXd J_temp;
    Eigen::Matrix3d R_wb1, R_wb2;
    Eigen::Vector3d p_wb1, p_wb2;
    double pos_diff = 0.;
    double ori_diff = 0.;
    for(int i=0; i<n_contact_; ++i)
    {      
        R_wb1 = robot_->getBodyNodeIsometry(link_idx_[i]).linear();
        p_wb1 = robot_->getBodyNodeIsometry(link_idx_[i]).translation();

        R_wb2 = robot_temp_->getBodyNodeIsometry(link_idx_[i]).linear();
        p_wb2 = robot_temp_->getBodyNodeIsometry(link_idx_[i]).translation();
        J_temp = robot_temp_->getBodyNodeJacobian(link_idx_[i]);
        // double det2 = (J_temp * J_temp.transpose()).determinant();
        // std::cout<<"det(" << i << ") =" << det2 << std::endl;

        pos_diff += (p_wb1-p_wb2).norm();
        ori_diff += (R_wb1.transpose()*R_wb2 - Eigen::MatrixXd::Identity(3,3)).diagonal().norm();
    }

    Eigen::MatrixXd Jcom_temp = robot_temp_->getCoMJacobian() * Nc_; // 6 X ndof 
    // Eigen::MatrixXd zz = Jcom_temp * Jcom_temp.transpose();
    // double det = (Jcom_temp).determinant();
    // my_utils::pretty_print(zz, std::cout, "zz");
    // std::cout<<"det = " << det << std::endl;
    Eigen::JacobiSVD< Eigen::MatrixXd > svd(Jcom_temp, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    // if(cond > 15.){
    //     std::cout<<"svd = ";
    //     for(int i = 0; i<svd.singularValues().size(); ++i){
    //         std::cout<<svd.singularValues()(i) << ", ";
    //     }
    //     std::cout<<"cond = " << cond << std::endl;
    // }
    if(pos_diff > 1. || ori_diff > 0.1 || cond>14.) {
        com_pos_sample = com_pos_ini;
        // q_sample = Eigen::VectorXd::Zero(q.size());
    }  
    else {
        int n=10; double n_inv = 1./((double)n);
        com_pos_sample = com_pos_ini;
        q = q_curr;
        for(int i=0;i<n;i++) {
            q += dq * n_inv;         
            robot_temp_->updateSystem(q, qdot, false);  
            Jcom_temp = robot_temp_->getCoMJacobian(); 
            com_pos_sample += Jcom_temp * Nc_ * dq * n_inv;
            // q_sample = Eigen::VectorXd::Zero(q.size());
        }
        // com_pos_sample = com_pos_ini + Jcom_* Nc_ * dq;
    }
}



void MagnetoGIWCPlanner::_buildCoMJacobian()
{
    Eigen::MatrixXd Jtmp = robot_->getCoMJacobian();  // 6 X ndof 
    // std::cout<< "COM Jacobian size: " << Jtmp.rows() << ", " << Jtmp.cols() << std::endl;
    Jcom_ = Jtmp.block(3, 0, 3, robot_->getNumDofs());
    // my_utils::pretty_print(Jcom_, std::cout, "Jcom_");
    n_dof_ = Jcom_.cols();
}

void MagnetoGIWCPlanner::_buildContactJacobianNullspace(const std::vector<ContactSpec*> & contact_list)
{
    Eigen::MatrixXd Jc;
    if(contact_list.empty())    
        Nc_ = Eigen::MatrixXd::Identity(n_dof_, n_dof_); // num_qdot_ : num active joint    
    else {
        _buildJacobianFromContacts(contact_list, Jc);
        _buildProjectionMatrix(Jc, Nc_);
    }
    
    // my_utils::pretty_print(Nc_, std::cout, "Nc_");    
}

void MagnetoGIWCPlanner::_buildJacobianFromContacts(const std::vector<ContactSpec*> & contact_list,
                                    Eigen::MatrixXd& Jc) {

        contact_list[0]->getContactJacobian(Jc);
        
        Eigen::MatrixXd Jc_i;
        int num_rows , num_new_rows, num_qdot_ = Jc.cols();
        for (int i(1); i < contact_list.size(); ++i) {
            contact_list[i]->getContactJacobian(Jc_i);
            num_rows = Jc.rows();
            num_new_rows = Jc_i.rows();
            Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
            Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
        } 
}

void MagnetoGIWCPlanner::_buildProjectionMatrix(const Eigen::MatrixXd& J,
                                            Eigen::MatrixXd& N) {    
    int num_qdot_ = J.cols();  
    Eigen::MatrixXd J_pinv = _pinv(J);
    Eigen::MatrixXd I_mtx = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);

    N = I_mtx - J_pinv * J;
}

void MagnetoGIWCPlanner::_buildContactPlane(const std::vector<ContactSpec*> & contact_list)
{
    // build abc_contact_, d_contact_ : ax + by + cz + d < 0
    int link_idx;
    int dim_col_i = 3; // x,y,z
    abc_contact_ = Eigen::MatrixXd::Zero(n_contact_, dim_col_i);
    d_contact_ = Eigen::VectorXd::Zero(n_contact_);

    Eigen::Vector3d normal;
    for(int i=0; i<n_contact_; ++i)
    {
        link_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
        // std::cout<<"link_idx : " <<link_idx << std::endl;
        Eigen::Matrix3d R_wb = robot_->getBodyNodeIsometry(link_idx).linear();
        Eigen::Vector3d p_wb = robot_->getBodyNodeIsometry(link_idx).translation();
        normal = R_wb.transpose().row(2); // z-dir : normalvector
        abc_contact_.row(i) = -normal;
        d_contact_(i) = p_wb.transpose()*normal;
    }
    // my_utils::pretty_print(abc_contact_, std::cout, "abc_contact_");
    // my_utils::pretty_print(d_contact_, std::cout, "d_contact_");
}

void MagnetoGIWCPlanner::_buildAstanceMatrix(const std::vector<ContactSpec*> & contact_list)
{    
    switch(contact_type_) // POINT, SURFACE, FIXED, SURFACE_BODYFRAME, POINT_BODYFRAME
    {
        case ContactSpecType::POINT :
        case ContactSpecType::POINT_BODYFRAME :
            // std::cout<<"POINT contact" << std:: endl;
            _buildAstanceMatrixPoint(contact_list);
            break;
        case ContactSpecType::SURFACE :
        case ContactSpecType::SURFACE_BODYFRAME :
            // std::cout<<"SURFACE contact" << std:: endl;
            _buildAstanceMatrixSurface(contact_list);
            break;
    }
    
    // my_utils::pretty_print(Astance_, std::cout, "Astance_");
}

void MagnetoGIWCPlanner::_buildAstanceMatrixPoint(const std::vector<ContactSpec*> & contact_list)
{
    Fm_over_m_ = Eigen::VectorXd::Zero(3*n_contact_);
    Astance_ = Eigen::MatrixXd::Zero(6, 3*n_contact_);
    Eigen::MatrixXd Mat_AdT = Eigen::MatrixXd::Zero(6,3);

    int link_idx;
    for(int i(0); i<n_contact_; ++i)
    {        
        link_idx = ((BodyFramePointContactSpec*)contact_list[i])->getLinkIdx();
        // std::cout<<"link_idx : " <<link_idx << std::endl;
        Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(link_idx).linear();
        Eigen::VectorXd p_wb = robot_->getBodyNodeIsometry(link_idx).translation();
        Eigen::VectorXd p_com = robot_->getCoMPosition();

        // my_utils::pretty_print(p_wb, std::cout, "p_wb");        
        // p_wb = p_wb - p_com;
        // p_wb = Eigen::VectorXd::Zero(3);
        
        Eigen::MatrixXd p_skew = Eigen::MatrixXd::Zero(3,3);
        // [[ 0, -3,  2],
        // [ 3,  0, -1],
        // [-2,  1,  0]]
        p_skew <<  0.0,     -p_wb(2), p_wb(1),
                   p_wb(2), 0.0     , -p_wb(0),
                  -p_wb(1), p_wb(0) , 0.0;
               
        Mat_AdT.block(0,0,3,3) = -p_skew*R_wb;
        Mat_AdT.block(3,0,3,3) = -R_wb; 
        
        Astance_.block(0,3*i,6,3) = Mat_AdT;
        Fm_over_m_[ 3*i + contact_list[i]->getFzIndex() ] = -fm_over_m_;
    }    
}

void MagnetoGIWCPlanner::_buildAstanceMatrixSurface(const std::vector<ContactSpec*> & contact_list)
{
    // int contact_dim = 6;
    Fm_over_m_ = Eigen::VectorXd::Zero(6*n_contact_);
    Astance_ = Eigen::MatrixXd::Zero(6, 6*n_contact_);
    Eigen::MatrixXd Mat_AdT = Eigen::MatrixXd::Zero(6,6);

    int link_idx;
    for(int i(0); i<n_contact_; ++i)
    {        
        link_idx = ((BodyFrameSurfaceContactSpec*)contact_list[i])->getLinkIdx();
        // std::cout<<"link_idx : " <<link_idx << std::endl;
        Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(link_idx).linear();
        Eigen::VectorXd p_wb = robot_->getBodyNodeIsometry(link_idx).translation();
        Eigen::VectorXd p_com = robot_->getCoMPosition();

        my_utils::pretty_print(p_wb, std::cout, "p_wb");
        // p_wb = p_wb - p_com;
        // p_wb = Eigen::VectorXd::Zero(3);
        
        Eigen::MatrixXd p_skew = Eigen::MatrixXd::Zero(3,3);
        // [[ 0, -3,  2],
        // [ 3,  0, -1],
        // [-2,  1,  0]]
        p_skew <<  0.0,     -p_wb(2), p_wb(1),
                   p_wb(2), 0.0     , -p_wb(0),
                  -p_wb(1), p_wb(0) , 0.0;

        Mat_AdT.block(0,0,3,3) = -R_wb;
        Mat_AdT.block(3,3,3,3) = -R_wb;        
        Mat_AdT.block(0,3,3,3) = -p_skew*R_wb;
        
        Astance_.block(0,6*i,6,6) = Mat_AdT;
        Fm_over_m_[ 6*i + contact_list[i]->getFzIndex() ] = -fm_over_m_;
    }
}

void MagnetoGIWCPlanner::_updateConvexHull()
{
    // check empty
    // std::cout<<"i'm here : size of Uf_ is "<< Uf_.rows() << "," << Uf_.cols() << std::endl;
    if(Uf_.rows() == 0 || Uf_.cols() == 0)
        return;

    // double description convex hull
    Polyhedron poly_1, poly_2;
    // 1. face(Uf_)->span(Uf_S=Vf_)
    bool b_poly_Uf = poly_1.setHrep(Uf_, Eigen::VectorXd::Zero(Uf_.rows()));
    Eigen::MatrixXd Vf = (poly_1.vrep().first).transpose();

    // 2. span(A_stance * Vf_)->face(U_st)
    // poly.vrep() return VrepXd:(first:Matrix V, second:Vector index(0:Ray, 1:vertex)) 
    Eigen::MatrixXd Vst = Astance_ * Vf;
    bool b_poly_Vst = poly_2.setVrep( Vst.transpose(), Eigen::VectorXd::Zero(Vst.cols()) );
    Ust_ = poly_2.hrep().first;

    //check     
    // Eigen::MatrixXd XX = Ust * Vst; 

}

void MagnetoGIWCPlanner::_buildHyperPlane(Eigen::MatrixXd& abc, Eigen::VectorXd& d) {
    Eigen::MatrixXd abc_i; 
    Eigen::VectorXd d_i;

    int dim_row_i = Ust_.rows();
    int dim_col_i = 3; // x,y,z

    abc = Eigen::MatrixXd::Zero(5*dim_row_i,dim_col_i);
    d = Eigen::VectorXd::Zero(5*dim_row_i);

    Eigen::MatrixXd grav_i = Eigen::MatrixXd::Zero(5,3);
    double g_perturb;
    g_perturb = 2; //  = 0.2
    grav_i <<   0.0,  0.0, -9.81,
             g_perturb,  0.0, -9.81,
            -g_perturb,  0.0, -9.81,
             0.0,  g_perturb, -9.81,
             0.0, -g_perturb, -9.81;
    Eigen::VectorXd gravity = Eigen::VectorXd::Zero(3);

    for(int i(0); i<grav_i.rows(); ++i) 
    {        
        gravity = grav_i.row(i);
        // my_utils::pretty_print(gravity, std::cout, "gravity"); 
        _buildHyperPlane(gravity, abc_i, d_i);
        abc.block(i*dim_row_i, 0, dim_row_i, dim_col_i) = abc_i;
        d.segment(i*dim_row_i, dim_row_i) = d_i;
    }

    
    // add contact plane : abc_contact_ , d_contact_
    abc.conservativeResize(5*dim_row_i + abc_contact_.rows(),dim_col_i);
    abc.block(5*dim_row_i,0,abc_contact_.rows(),dim_col_i) = abc_contact_;
    d.conservativeResize(5*dim_row_i + d_contact_.size());
    d.segment(5*dim_row_i,d_contact_.size()) = d_contact_;
}

 void MagnetoGIWCPlanner::_buildHyperPlane(const::Eigen::MatrixXd& gravity, Eigen::MatrixXd& abc, Eigen::VectorXd& d)
 {
    Eigen::MatrixXd G_x = Eigen::MatrixXd::Zero(3,3);
    G_x << 0.0, -gravity(2), gravity(1),
            gravity(2), 0.0, -gravity(0),
            -gravity(1), gravity(0), 0.0;

    Eigen::MatrixXd Ust1 = Ust_.leftCols(3);
    Eigen::MatrixXd Ust2 = Ust_.rightCols(3);

    // abc*[x;y;z] < -d : ax+by+cz+d < 0 
    abc = -Ust1*G_x;
    d = - ( Ust_*Astance_*Fm_over_m_ - Ust2*gravity );
 }

double MagnetoGIWCPlanner::_getMinDistance(const Eigen::VectorXd& pos, const Eigen::MatrixXd& abc, const Eigen::VectorXd& d)
{
    // pos : 3x1
    // abc : mx3
    // d : mx1
    // dist_list = -(a*x+b*y+c*z+d)./sqrt(a.*a + b.*b + c.*c);
    // mind = min(dist_list);

    double num, den, dist, min_dist=1e5;
    for (int i=0; i<abc.rows(); ++i)
    {
        Eigen::VectorXd abc_i = abc.row(i);

        num = abc_i.dot(pos) + d(i);
        den = abc_i.norm();

        dist = - num / den;
        if(min_dist > dist)
            min_dist = dist;

    }
    return min_dist;
    
}


Eigen::MatrixXd MagnetoGIWCPlanner::_null(const Eigen::MatrixXd& input)
{
    Eigen::MatrixXd ret =  my_utils::getNullSpace(input, 0.0001);
    return ret;
}

Eigen::MatrixXd MagnetoGIWCPlanner::_pinv(const Eigen::MatrixXd& input)
{
    Eigen::MatrixXd ret;
    my_utils::pseudoInverse(input, 0.0001, ret);
    return ret;
}

void MagnetoGIWCPlanner::_ineqMat2Uf(Eigen::MatrixXd &Uf) {
    Uf = -Uf.topLeftCorner(Uf.rows()-1, Uf.cols());
}
 