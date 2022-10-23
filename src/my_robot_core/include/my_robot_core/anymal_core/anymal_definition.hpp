#pragma once
#include <string>

namespace ANYmal {
constexpr int n_bodynode = 86; //include world
constexpr int n_leg = 4;

constexpr int n_adof = 18; // 18

// qdot, qddot, tau_act
constexpr int n_vdof = 6;
constexpr int n_dof = n_vdof + n_adof; // 24 (=nv,nu)
constexpr int idx_vdof [n_vdof] = {0,1,2,3,4,5};
constexpr int idx_adof [n_adof] = {6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};

// q
constexpr int idx_vdof_config [n_vdof+1] = {0,1,2,3,4,5,6};
constexpr int idx_adof_config [n_adof] = {7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24};

}  // namespace ANYmal

namespace ANYmalLegJointType { 
    constexpr int HAA = 0;
    constexpr int HFE = 1; 
    constexpr int KFE = 2;
    constexpr int idx_haa[4] = {6, 9, 12, 15};
    constexpr int idx_hfe[4] = {7, 10, 13, 16};
    constexpr int idx_kfe[4] = {8, 11, 14, 17};
}

// 0:LF 1:LH 2:RF 3:RH
namespace ANYmalFoot {
const std::string Names[4] = {
    "LF", "LH", "RF", "RH" };

const std::string NamesLower[4] = {
    "lf", "lh", "rf", "rh" };

constexpr int LinkIdx[4] = {
    22, // LF_FOOT
    42, // LH_FOOT
    62, // RF_FOOT
    82 // RH_FOOT
}; // link in contact

constexpr int LF = 0;
constexpr int LH = 1;
constexpr int RF = 2;
constexpr int RH = 3;
}// namespace ANYmalFoot

namespace ANYmalEE {
constexpr int EEarm = 172;//ur3_ee_link = 172;
}

namespace ANYmalBodyNode {
constexpr int base = 2;
constexpr int LF_HAA = 4;
constexpr int LF_HIP = 6;
constexpr int LF_hip_fixed = 8;
constexpr int LF_HFE = 10;
constexpr int LF_THIGH = 12;
constexpr int LF_thigh_fixed = 14;
constexpr int LF_KFE = 16;
constexpr int LF_SHANK = 18;
constexpr int LF_shank_fixed = 20;
constexpr int LF_FOOT = 22;
constexpr int LH_HAA = 24;
constexpr int LH_HIP = 26;
constexpr int LH_hip_fixed = 28;
constexpr int LH_HFE = 30;
constexpr int LH_THIGH = 32;
constexpr int LH_thigh_fixed = 34;
constexpr int LH_KFE = 36;
constexpr int LH_SHANK = 38;
constexpr int LH_shank_fixed = 40;
constexpr int LH_FOOT = 42;
constexpr int RF_HAA = 44;
constexpr int RF_HIP = 46;
constexpr int RF_hip_fixed = 48;
constexpr int RF_HFE = 50;
constexpr int RF_THIGH = 52;
constexpr int RF_thigh_fixed = 54;
constexpr int RF_KFE = 56;
constexpr int RF_SHANK = 58;
constexpr int RF_shank_fixed = 60;
constexpr int RF_FOOT = 62;
constexpr int RH_HAA = 64;
constexpr int RH_HIP = 66;
constexpr int RH_hip_fixed = 68;
constexpr int RH_HFE = 70;
constexpr int RH_THIGH = 72;
constexpr int RH_thigh_fixed = 74;
constexpr int RH_KFE = 76;
constexpr int RH_SHANK = 78;
constexpr int RH_shank_fixed = 80;
constexpr int RH_FOOT = 82;
constexpr int battery = 84;
constexpr int bottom_shell = 86;
constexpr int face_front = 88;
constexpr int depth_camera_front_camera = 90;
constexpr int depth_camera_front_camera_parent = 92;
constexpr int depth_camera_front_color_frame = 94;
constexpr int depth_camera_front_color_optical_frame = 96;
constexpr int depth_camera_front_depth_optical_frame = 98;
constexpr int wide_angle_camera_front_camera = 100;
constexpr int wide_angle_camera_front_camera_parent = 102;
constexpr int face_rear = 104;
constexpr int depth_camera_rear_camera = 106;
constexpr int depth_camera_rear_camera_parent = 108;
constexpr int depth_camera_rear_color_frame = 110;
constexpr int depth_camera_rear_color_optical_frame = 112;
constexpr int depth_camera_rear_depth_optical_frame = 114;
constexpr int wide_angle_camera_rear_camera = 116;
constexpr int wide_angle_camera_rear_camera_parent = 118;
constexpr int handle = 120;
constexpr int hatch = 122;
constexpr int remote = 124;
constexpr int base_inertia = 126;
constexpr int depth_camera_left_camera = 128;
constexpr int depth_camera_left_camera_parent = 130;
constexpr int depth_camera_left_color_frame = 132;
constexpr int depth_camera_left_color_optical_frame = 134;
constexpr int depth_camera_left_depth_optical_frame = 136;
constexpr int depth_camera_right_camera = 138;
constexpr int depth_camera_right_camera_parent = 140;
constexpr int depth_camera_right_color_frame = 142;
constexpr int depth_camera_right_color_optical_frame = 144;
constexpr int depth_camera_right_depth_optical_frame = 146;
constexpr int docking_hatch_cover = 148;
constexpr int lidar_cage = 150;
constexpr int lidar = 152;
constexpr int top_shell = 154;
constexpr int imu_link = 156;
constexpr int ur3_base = 158;
constexpr int ur3_shoulder_link = 160;
constexpr int ur3_upper_arm_link = 162;
constexpr int ur3_forearm_link = 164;
constexpr int ur3_wrist_1_link = 166;
constexpr int ur3_wrist_2_link = 168;
constexpr int ur3_wrist_3_link = 170;
constexpr int ur3_ee_link = 172;
}  // namespace ANYmalBodyNode

namespace ANYmalDoF {
// universe:0, root_joint:1
constexpr int LF_HAA = 2;
constexpr int LF_HFE = 3;
constexpr int LF_KFE = 4;
constexpr int LH_HAA = 5;
constexpr int LH_HFE = 6;
constexpr int LH_KFE = 7;
constexpr int RF_HAA = 8;
constexpr int RF_HFE = 9;
constexpr int RF_KFE = 10;
constexpr int RH_HAA = 11;
constexpr int RH_HFE = 12;
constexpr int RH_KFE = 13;
constexpr int ur3_shoulder_pan_joint = 14;
constexpr int ur3_shoulder_lift_joint = 15;
constexpr int ur3_elbow_joint = 16;
constexpr int ur3_wrist_1_joint = 17;
constexpr int ur3_wrist_2_joint = 18;
constexpr int ur3_wrist_3_joint = 19;

}  // namespace ANYmalDoF

