#ifndef COB_OMNI_DRIVE_CONTROLLER_VELOCITY_ESTIMATOR_H_
#define COB_OMNI_DRIVE_CONTROLLER_VELOCITY_ESTIMATOR_H_

#include <limits>
#include <cmath>


class VelocityEstimator {
    double overflow_;
    double overflow_twice_;
    double last_pos_;
    double norm(double val){
        while(val >= overflow_){
            val -= overflow_twice_;
        }
        while(val < -overflow_){
            val += overflow_twice_;
        }
        return val;
    }
public:
    VelocityEstimator(double overflow) : overflow_(overflow), overflow_twice_(overflow*2) {  reset(); }
    double estimateVelocity(double pos, double vel, double period) {
        if(period > 0 && !std::isnan(last_pos_)){
            if(overflow_ > 0) vel = norm(pos - last_pos_) / period; // derive without overflow correction
            else if(overflow_ < 0) vel = (pos - last_pos_) / period; // derive only
            // else turn off estimation
        }
        last_pos_ = pos;
        return vel;
    }
    void reset() { last_pos_ = std::numeric_limits<double>::quiet_NaN(); }
};

#endif
