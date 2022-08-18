#include <my_wbc/WBLC/KinWBC.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/pseudo_inverse.hpp>

KinWBC::KinWBC(const std::vector<bool>& act_joint)
    : num_act_joint_(0),
      threshold_(0.001)
    // threshold_(0.005)
    // threshold_(0.0005)
{
    my_utils::pretty_constructor(3, "Kin WBC");
    num_qdot_ = act_joint.size();

    act_jidx_.clear();
    for (int i(0); i < num_qdot_; ++i) {
        if (act_joint[i]) {
            act_jidx_.push_back(i);
            ++num_act_joint_;
        }
    }
    // my_utils::pretty_print(act_jidx_, "act jidx");
    I_mtx = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
}

bool KinWBC::FindConfigurationContactPriority(const Eigen::VectorXd & curr_config,
                                            const std::vector<Task*> & task_list,
                                            const std::vector<ContactSpec*> & contact_list,
                                            Eigen::VectorXd & jpos_cmd,
                                            Eigen::VectorXd & jvel_cmd,
                                            Eigen::VectorXd & jacc_cmd) {
    // set contact priority 
    Eigen::MatrixXd Jc, Nc;
    if(contact_list.empty())    
        Nc = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_); // num_qdot_ : num active joint    
    else {
        _BuildJacobianFromContacts(contact_list, Jc);
        _BuildProjectionMatrix(Jc, Nc);
    }


    Eigen::VectorXd delta_q, qdot, qddot;
    Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;


}


bool KinWBC::FindConfiguration(const Eigen::VectorXd& curr_config,
                               const std::vector<Task*>& task_list,
                               const std::vector<ContactSpec*>& contact_list,
                               Eigen::VectorXd& jpos_cmd,
                               Eigen::VectorXd& jvel_cmd,
                               Eigen::VectorXd& jacc_cmd) {
    // printf("contact list size: %d\n", contact_list.size());
    // Contact Jacobian Setup
    // Pmx
    Eigen::MatrixXd Jc, Nc;
    if(contact_list.empty())    
        Nc = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_); // num_qdot_ : num active joint    
    else {
        _BuildJacobianFromContacts(contact_list, Jc);
        _BuildProjectionMatrix(Jc, Nc);
    }   

    Eigen::VectorXd delta_q, qdot, qddot, JtDotQdot;
    Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    // First Task
    Task* task = task_list[0];
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    JtPre = Jt * Nc;

    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q = JtPre_pinv * (task->pos_err);

    // //0112 my_utils::saveVector(delta_q, "delta_q0");
    // //0112 my_utils::saveVector(task->pos_err, "delta_x0");

    qdot = JtPre_pinv * (task->vel_des);
    qddot = JtPre_pinv * (task->acc_des - JtDotQdot);
    // qddot = JtPre_pinv * (task->op_cmd - JtDotQdot);

    Eigen::VectorXd prev_delta_q = delta_q;
    Eigen::VectorXd prev_qdot = qdot;
    Eigen::VectorXd prev_qddot = qddot;

    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;

    // my_utils::color_print(myColor::Red, "======== 0 ========");
    // Eigen::VectorXd xdot_c = Jc * delta_q;
    // my_utils::pretty_print(xdot_c, std::cout, "contact vel");
    // my_utils::pretty_print(Jt, std::cout, "task Jt");
    // my_utils::pretty_print(Jc, std::cout, "Jc");
    // my_utils::pretty_print(Nc, std::cout, "Nc");
    // my_utils::pretty_print(JtPre, std::cout, "JtNc");
    // my_utils::pretty_print(JtPre_pinv, std::cout, "JtPre_inv");
    // my_utils::pretty_print(task->pos_err, std::cout, "pos_err");
    // my_utils::pretty_print(task->vel_des, std::cout, "vel_des");
    // my_utils::pretty_print(delta_q, std::cout, "delta q");
    // my_utils::pretty_print(qdot, std::cout, "qdot");
    // my_utils::pretty_print(qddot, std::cout, "qddot");
    // Eigen::MatrixXd test = Jt * N_pre;
    // my_utils::pretty_print(test, std::cout, "Jt1N1");
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
    // JtPre, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // std::cout << "svd" << std::endl;
    // std::cout << svd1.singularValues() << std::endl;

    for (int i(1); i < task_list.size(); ++i) {
        task = task_list[i];

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        JtPre = Jt * N_pre;

        _PseudoInverse(JtPre, JtPre_pinv);
        delta_q =
            prev_delta_q + JtPre_pinv * (task->pos_err - Jt * prev_delta_q);
        // //0112 my_utils::saveVector(delta_q, "delta_q" + std::to_string(i));
        // //0112 my_utils::saveVector(task->pos_err, "delta_x" + std::to_string(i));

        // for(int j=0; j<JtPre_pinv.rows(); ++j)  {
        //     //0112 my_utils::saveVector(JtPre_pinv.row(j), "JtPre_pinv"+ std::to_string(i) );
        // }


        qdot = prev_qdot + JtPre_pinv * (task->vel_des - Jt * prev_qdot);
        qddot = prev_qddot +
                    JtPre_pinv * (task->acc_des - JtDotQdot - Jt * prev_qddot);
        // qddot = prev_qddot +
        //         JtPre_pinv * (task->op_cmd - JtDotQdot - Jt * prev_qddot);

        // my_utils::color_print(myColor::Red,
        //"======== " + std::to_string(i) + " ========");
        // my_utils::pretty_print(Jt, std::cout, "Jt");
        // my_utils::pretty_print(N_pre, std::cout, "N_pre");
        // my_utils::pretty_print(JtPre, std::cout, "JtPre");
        // my_utils::pretty_print(JtPre_pinv, std::cout, "JtPre_inv");
        // my_utils::pretty_print(task->pos_err, std::cout, "pos_err");
        // my_utils::pretty_print(task->vel_des, std::cout, "vel_des");
        // my_utils::pretty_print(delta_q, std::cout, "delta q");
        // my_utils::pretty_print(qdot, std::cout, "qdot");
        // xdot_c = Jc * delta_q;
        // my_utils::pretty_print(xdot_c, std::cout, "contact vel");
        // Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
        // JtPre, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // std::cout << "svd" << std::endl;
        // std::cout << svd2.singularValues() << std::endl;

        // For the next task
        _BuildProjectionMatrix(JtPre, N_nx);
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot = qdot;
        prev_qddot = qddot;

    }
    // xdot_c = Jc * delta_q;
    // my_utils::pretty_print(xdot_c, std::cout, "contact vel");
    for (int i(0); i < num_act_joint_; ++i) {
        jpos_cmd[i] = curr_config[act_jidx_[i]] + delta_q[act_jidx_[i]];
        jvel_cmd[i] = qdot[act_jidx_[i]];
        jacc_cmd[i] = qddot[act_jidx_[i]];
    }
    
    // my_utils::saveVector(jpos_des_, "jpos_des");
    // my_utils::saveVector(jvel_des_, "jvel_des");
    return true;
}

bool KinWBC::FindFullConfiguration(const Eigen::VectorXd& curr_config,
                                const std::vector<Task*>& task_list,
                                const std::vector<ContactSpec*>& contact_list,
                                Eigen::VectorXd& jpos_cmd,
                                Eigen::VectorXd& jvel_cmd,
                                Eigen::VectorXd& jacc_cmd) {

    Eigen::MatrixXd Jc, Nc, Jc_pinv;
    Eigen::VectorXd JcDotQdot, JcpinvJcDotQdot;
    if(contact_list.empty()){
        Nc = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_); // num_qdot_ : num active joint
        JcpinvJcDotQdot = Eigen::VectorXd::Zero(num_qdot_);
    } else {  
        _BuildJacobianFromContacts(contact_list, Jc);
        _BuildJdotQdotFromContacts(contact_list, JcDotQdot);
        _PseudoInverse(Jc, Jc_pinv);
        JcpinvJcDotQdot = Jc_pinv*JcDotQdot;
        _BuildProjectionMatrix(Jc, Nc);    
    }
    

    Eigen::VectorXd delta_q, qdot, qddot, JtDotQdot;
    Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    // First Task
    Task* task = task_list[0];
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    JtPre = Jt * Nc;

    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q = JtPre_pinv * (task->pos_err);
    qdot = JtPre_pinv * (task->vel_des);
    // qddot = JtPre_pinv * (task->acc_des - JtDotQdot);
    // qddot = JtPre_pinv * (task->acc_des + Jt * JcpinvJcDotQdot - JtDotQdot); // modified 2021.1.23
    qddot = - JcpinvJcDotQdot + JtPre_pinv * (task->acc_des - JtDotQdot); // modified 2021.2.7
    // qddot = JtPre_pinv * (task->op_cmd - JtDotQdot);

    Eigen::VectorXd xdot_c = Jc * delta_q;
    my_utils::saveVector(delta_q, "delta_q0");
    my_utils::saveVector(task->pos_err, "delta_x0");
    my_utils::saveVector(xdot_c, "xdot_c0");

    Eigen::VectorXd prev_delta_q = delta_q;
    Eigen::VectorXd prev_qdot = qdot;
    Eigen::VectorXd prev_qddot = qddot;

    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;

    for (int i(1); i < task_list.size(); ++i) {
        task = task_list[i];

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        JtPre = Jt * N_pre;

        _PseudoInverse(JtPre, JtPre_pinv);
        delta_q =
            prev_delta_q + JtPre_pinv * (task->pos_err - Jt * prev_delta_q);
        
        qdot = prev_qdot + JtPre_pinv * (task->vel_des - Jt * prev_qdot);
        qddot = prev_qddot +
                    JtPre_pinv * (task->acc_des - JtDotQdot - Jt * prev_qddot);

        my_utils::saveVector(delta_q, "delta_q" + std::to_string(i) + "_" + std::to_string(task_list.size()) );
        my_utils::saveVector(task->pos_err, "delta_x" + std::to_string(i) + "_" + std::to_string(task_list.size()) );
        xdot_c = Jc * delta_q;
        my_utils::saveVector(xdot_c, "xdot_c"+ std::to_string(i) + "_" + std::to_string(task_list.size()) );

        // For the next task
        _BuildProjectionMatrix(JtPre, N_nx);
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot = qdot;
        prev_qddot = qddot;
    }

    jpos_cmd = curr_config + delta_q;
    jvel_cmd = qdot;
    jacc_cmd = qddot;

    my_utils::saveVector(jpos_cmd, "jpos_des_full");
    my_utils::saveVector(jvel_cmd, "jvel_des_full");

    return true;    
}

void KinWBC::_BuildJacobianFromContacts(const std::vector<ContactSpec*> & contact_list,
                                    Eigen::MatrixXd& Jc) {

    contact_list[0]->getContactJacobian(Jc);
    Eigen::MatrixXd Jc_i;
    int num_rows , num_new_rows;
    for (int i(1); i < contact_list.size(); ++i) {
        contact_list[i]->getContactJacobian(Jc_i);
        num_rows = Jc.rows();
        num_new_rows = Jc_i.rows();
        Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
        Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
    } 
}

void KinWBC::_BuildJdotQdotFromContacts(const std::vector<ContactSpec*> & contact_list,
                                    Eigen::VectorXd& JcDotQdot){

    contact_list[0]->getJcDotQdot(JcDotQdot);
    Eigen::VectorXd Jc_i;
    int num_rows , num_new_rows;
    for (int i(1); i < contact_list.size(); ++i) {
        contact_list[i]->getJcDotQdot(Jc_i);
        num_rows = JcDotQdot.size();
        num_new_rows = Jc_i.size();
        JcDotQdot.conservativeResize(num_rows + num_new_rows);
        JcDotQdot.segment(num_rows, num_new_rows) = Jc_i;
    } 
}


void KinWBC::_BuildProjectionMatrix(const Eigen::MatrixXd& J,
                                    Eigen::MatrixXd& N) {
    Eigen::MatrixXd J_pinv;
    _PseudoInverse(J, J_pinv);
    N = I_mtx - J_pinv * J;
}

void KinWBC::_PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd& Jinv) {
    my_utils::pseudoInverse(J, threshold_, Jinv);

    // mx Lambda_inv = J * Ainv_ * J.transpose();
    // mx Lambda;
    // dynacore::pseudoInverse(Lambda_inv, threshold_, Lambda);
    // Jinv = Ainv_ * J.transpose() * Lambda;
}
