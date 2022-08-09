#pragma once
#include <string>

namespace ANYmal {
constexpr int n_bodynode = 91;

constexpr int n_leg = 4;
constexpr int n_leg_adof = 3;
constexpr int n_legs_adof = n_leg*n_leg_adof ;
constexpr int n_arm_adof = 6;

constexpr int n_vdof = 6;
constexpr int n_adof = n_legs_adof + n_arm_adof;
constexpr int n_dof = n_vdof + n_adof;

constexpr int idx_vdof [n_vdof] = {0,1,2,3,4,5};
constexpr int idx_adof [n_adof] = {6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
constexpr int idx_leg_adof [n_adof] = {6,7,8, 9,10,11, 12,13,14, 15,16,17};
constexpr int idx_arm_adof [n_adof] = {18,19,20,21,22,23};
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
    15, // LF_FOOT
    25, // LH_FOOT
    35, // RF_FOOT
    45 // RH_FOOT
}; // link in contact

constexpr int LF = 0;
constexpr int LH = 1;
constexpr int RF = 2;
constexpr int RH = 3;
}// namespace ANYmalFoot

namespace ANYmalEE {
constexpr int EEarm = 90;//ur3_ee_link = 90;
}

namespace ANYmalBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int base = 5;
constexpr int LF_HAA = 6;
constexpr int LF_HIP = 7;
constexpr int LF_hip_fixed = 8;
constexpr int LF_HFE = 9;
constexpr int LF_THIGH = 10;
constexpr int LF_thigh_fixed = 11;
constexpr int LF_KFE = 12;
constexpr int LF_SHANK = 13;
constexpr int LF_shank_fixed = 14;
constexpr int LF_FOOT = 15;
constexpr int LH_HAA = 16;
constexpr int LH_HIP = 17;
constexpr int LH_hip_fixed = 18;
constexpr int LH_HFE = 19;
constexpr int LH_THIGH = 20;
constexpr int LH_thigh_fixed = 21;
constexpr int LH_KFE = 22;
constexpr int LH_SHANK = 23;
constexpr int LH_shank_fixed = 24;
constexpr int LH_FOOT = 25;
constexpr int RF_HAA = 26;
constexpr int RF_HIP = 27;
constexpr int RF_hip_fixed = 28;
constexpr int RF_HFE = 29;
constexpr int RF_THIGH = 30;
constexpr int RF_thigh_fixed = 31;
constexpr int RF_KFE = 32;
constexpr int RF_SHANK = 33;
constexpr int RF_shank_fixed = 34;
constexpr int RF_FOOT = 35;
constexpr int RH_HAA = 36;
constexpr int RH_HIP = 37;
constexpr int RH_hip_fixed = 38;
constexpr int RH_HFE = 39;
constexpr int RH_THIGH = 40;
constexpr int RH_thigh_fixed = 41;
constexpr int RH_KFE = 42;
constexpr int RH_SHANK = 43;
constexpr int RH_shank_fixed = 44;
constexpr int RH_FOOT = 45;
constexpr int battery = 46;
constexpr int bottom_shell = 47;
constexpr int face_front = 48;
constexpr int depth_camera_front_camera = 49;
constexpr int depth_camera_front_camera_parent = 50;
constexpr int depth_camera_front_color_frame = 51;
constexpr int depth_camera_front_color_optical_frame = 52;
constexpr int depth_camera_front_depth_optical_frame = 53;
constexpr int wide_angle_camera_front_camera = 54;
constexpr int wide_angle_camera_front_camera_parent = 55;
constexpr int face_rear = 56;
constexpr int depth_camera_rear_camera = 57;
constexpr int depth_camera_rear_camera_parent = 58;
constexpr int depth_camera_rear_color_frame = 59;
constexpr int depth_camera_rear_color_optical_frame = 60;
constexpr int depth_camera_rear_depth_optical_frame = 61;
constexpr int wide_angle_camera_rear_camera = 62;
constexpr int wide_angle_camera_rear_camera_parent = 63;
constexpr int handle = 64;
constexpr int hatch = 65;
constexpr int remote = 66;
constexpr int base_inertia = 67;
constexpr int depth_camera_left_camera = 68;
constexpr int depth_camera_left_camera_parent = 69;
constexpr int depth_camera_left_color_frame = 70;
constexpr int depth_camera_left_color_optical_frame = 71;
constexpr int depth_camera_left_depth_optical_frame = 72;
constexpr int depth_camera_right_camera = 73;
constexpr int depth_camera_right_camera_parent = 74;
constexpr int depth_camera_right_color_frame = 75;
constexpr int depth_camera_right_color_optical_frame = 76;
constexpr int depth_camera_right_depth_optical_frame = 77;
constexpr int docking_hatch_cover = 78;
constexpr int lidar_cage = 79;
constexpr int lidar = 80;
constexpr int top_shell = 81;
constexpr int imu_link = 82;
constexpr int ur3_base = 83;
constexpr int ur3_shoulder_link = 84;
constexpr int ur3_upper_arm_link = 85;
constexpr int ur3_forearm_link = 86;
constexpr int ur3_wrist_1_link = 87;
constexpr int ur3_wrist_2_link = 88;
constexpr int ur3_wrist_3_link = 89;
constexpr int ur3_ee_link = 90;
}  // namespace ANYmalBodyNode

namespace ANYmalDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int baseRotX = 5;
constexpr int LF_HAA = 6;
constexpr int LF_HFE = 7;
constexpr int LF_KFE = 8;
constexpr int LH_HAA = 9;
constexpr int LH_HFE = 10;
constexpr int LH_KFE = 11;
constexpr int RF_HAA = 12;
constexpr int RF_HFE = 13;
constexpr int RF_KFE = 14;
constexpr int RH_HAA = 15;
constexpr int RH_HFE = 16;
constexpr int RH_KFE = 17;
constexpr int ur3_shoulder_pan_joint = 18;
constexpr int ur3_shoulder_lift_joint = 19;
constexpr int ur3_elbow_joint = 20;
constexpr int ur3_wrist_1_joint = 21;
constexpr int ur3_wrist_2_joint = 22;
constexpr int ur3_wrist_3_joint = 23;
}  // namespace ANYmalDoF

namespace ANYmalAux {
constexpr double servo_rate = 0.001;
}
