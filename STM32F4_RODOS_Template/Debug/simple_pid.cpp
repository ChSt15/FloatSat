#include "rodos.h"

#include "simple_pid.hpp"


using namespace VCTR::CTRL;

// Implementation of the SimplePID class.

SimplePID::SimplePID(float kp, float ki, float kd, float maxOutput) :
    kp_(kp),
    ki_(ki),
    kd_(kd),
    maxOutput_(maxOutput),
    integral_(0.0f),
    lastError_(0.0f),
    setPoint_(0.0f),
    lastTime_(0) {

}

void SimplePID::setSetPoint(float setPoint) {

    setPoint_ = setPoint;

}

float SimplePID::getSetPoint() const {

    return setPoint_;

}

float SimplePID::update(float state, int64_t stateTime) {

    float error = setPoint_ - state;
    float errorDerivative = (error - lastError_) * SECONDS/(stateTime - lastTime_);

    integral_ += error * (stateTime - lastTime_) / SECONDS;

    float output = kp_*error + ki_*integral_ + kd_*errorDerivative;

    lastError_ = error;
    lastTime_ = stateTime;

    //Limit the output and do integral anti windup using back calculation
    if (output > maxOutput_) {
        output = maxOutput_;
        integral_ = 0;
    } else if (output < -maxOutput_) {
        output = -maxOutput_;
        integral_ = 0;
    }

    return output;

}