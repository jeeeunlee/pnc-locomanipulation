#include <my_robot_core/magneto_core/magneto_motion_api.hpp>

MotionCommand::MotionCommand() {
    motion_sets_.clear();
}

MotionCommand::MotionCommand(int _moving_link_id,
                    const MOTION_DATA& _motion_data) {
    motion_sets_.clear();
    motion_sets_.insert(std::make_pair(_moving_link_id, _motion_data));
}

// function

void MotionCommand::add_com_motion(const POSE_DATA& _pose_del,
                                    double _motion_period) {
    MOTION_DATA motion_data = MOTION_DATA(_pose_del, _motion_period);
    motion_sets_.insert(std::make_pair(-1, motion_data));
}

void MotionCommand::add_motion(int _moving_link_id,
                    const MOTION_DATA& _motion_data) {
    motion_sets_.insert(std::make_pair(_moving_link_id, _motion_data));
}

void MotionCommand::clear_and_add_motion(int _moving_link_id,
                    const MOTION_DATA& _motion_data) {
    motion_sets_.clear();
    motion_sets_.insert(std::make_pair(_moving_link_id, _motion_data));
}

int MotionCommand::get_moving_foot() {
    // TODO LATER : if you want to give two feet swing together ?
    for(auto &it : motion_sets_) {
        // it: <LINKIDX, MOTION_DATA>
        if(it.first > 0) {
            return it.first;
        }
    }
    return -1;
}

bool MotionCommand::get_foot_motion_command(MOTION_DATA &_motion_data) {
    for(auto &[idx, motion_data] : motion_sets_) {
        if(idx > 0) {
            _motion_data = motion_data;
            return true;
        }
    }
    return false;
}

 bool MotionCommand::get_foot_motion_command(MOTION_DATA &_motion_data, TARGET_LINK_IDX &_idx) {
    for(auto &[idx, motion_data] : motion_sets_) {
        if(idx > 0) {
            _idx = idx;
            _motion_data = motion_data;
            return true;
        }
    }
    return false;
}

bool MotionCommand::get_com_motion_command(MOTION_DATA &_motion_data) {
    for(auto &[idx, motion_data] : motion_sets_) {
        if(idx < 0) {
            _motion_data = motion_data;
            return true;
        }
    }
    return false;
}
