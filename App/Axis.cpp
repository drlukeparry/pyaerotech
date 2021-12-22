#include <cassert>
#include <algorithm>
#include <exception>

#include <A3200AeroBasicCommands.h>

#include <App/Controller.h>

#include "Exception.h"
#include "Controller.h"
#include "Axis.h"

using namespace aerotech;

Axis::Axis() :  mState(AxisState::DISABLED),
                mLabel(""),
                mController(nullptr)
{
    // private constructor
}


Axis::Axis(A3200Controller::Ptr controller, uint16_t axis) : mId(axis),
                                                             mState(AxisState::DISABLED),
                                                             mController(controller)
{

}


Axis::~Axis()
{
    
}

uint32_t Axis::axisMask() const
{
    return (1 << mId);
}

void Axis::enable(const uint8_t taskId)
{
    this->mController->enable(Ptr(this), taskId);
    //A3200MotionEnable(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask() );
}

void Axis::disable(const uint8_t taskId)
{
    this->mController->disable(Ptr(this), taskId);
    //A3200MotionDisable(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask());
}

void Axis::abort()
{
    this->mController->abort(Ptr(this));
    //A3200MotionAbort(mController->handle(), (AXISMASK) axisMask());
}

void Axis::acknowlegeFault(const uint8_t taskId)
{
    this->mController->acknowlegeFault(Ptr(this), taskId);
}


/*
 * Position and control related commands
 */
uint64_t Axis::status()
{
    return 0;
}

bool Axis::isEnabled()
{
    // AXISSTATUS_Enabling
     double value;
     bool state = A3200StatusGetItem(mController->handle(), mAxisMask, STATUSITEM_AxisStatus, AXISSTATUS_Enabling, &value);

     if(!state) {
         throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
     }

     return value;
}

bool Axis::isHomed()
{     
    double value;    
    bool state = A3200StatusGetItem(mController->handle(), mAxisMask, STATUSITEM_AxisStatus, AXISSTATUS_Homed, &value);

    if(!state) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }

    return value;
}

bool Axis::isBlocked()
{
    double value;    
    bool state = A3200StatusGetItem(mController->handle(), mAxisMask, STATUSITEM_AxisStatus, AXISSTATUS_MotionBlocked, &value);

    if(!state) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }


    return value;    
}

void Axis::home(const uint8_t taskId)
{
    this->mController->home(Ptr(this), taskId);
    //A3200MotionHome(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask());
}

void Axis::move(const double position, const double speed, const uint8_t taskId)
{
    this->mController->move(Ptr(this), position, speed, taskId);

#if 0
    if(speed > 1e-6){
        A3200MotionMoveAbs(mController->handle(), (TASKID) taskId, (AXISINDEX) mId, position, speed);
    } else {

    }
#endif
}

void Axis::moveRel(const double distance, const double speed, const uint8_t taskId)
{
    double value = distance;

    bool state;

    if(speed > 1e-6){
        state = A3200MotionMoveInc(mController->handle(), (TASKID) taskId, (AXISINDEX) mId, distance, speed);
    } else {
        state = A3200MotionLinear(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask() , &value);
    }

    if(!state) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }

}

void Axis::setDefaultSpeed(double speed)
{
    // todo implement
    mDefaultSpeed = speed;
}


void Axis::wait(uint64_t timeout, bool inPosition)
{
    this->mController->wait(Ptr(this), timeout, inPosition);

#if 0
    WAITOPTION waitOption = WAITOPTION_InPosition;

    if(!inPosition)
        waitOption = WAITOPTION_MoveDone;

    A3200MotionWaitForMotionDone(mController->handle(), (AXISMASK) axisMask(), waitOption, timeout, NULL);
#endif
}

double Axis::position()
{
    return this->getStatus(STATUSITEM_PositionFeedback);
}

double Axis::velocity(const bool average)
{
    return (average) ? this->getStatus(STATUSITEM_VelocityFeedbackAverage) : this->getStatus(STATUSITEM_VelocityFeedback);
}

double Axis::current(const bool average)
{
    return (average) ? this->getStatus(STATUSITEM_CurrentFeedbackAverage) : this->getStatus(STATUSITEM_CurrentFeedback);
}

double Axis::getStatus(const STATUSITEM &item)
{
    double value;    
    bool state = A3200StatusGetItem(mController->handle(), mId, item, 0, &value);

    if(!state) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }

    return value;
}

void Axis::setRampMode(const RampMode rampMode, const uint8_t taskId)
{
    this->mController->setRampMode(Ptr(this), rampMode, taskId);

    //A3200MotionSetupRampModeAxis(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask(), (RAMPMODE) rampMode);
}

void Axis::setRampRate(const double rampRate, const uint8_t taskId)
{
    this->mController->setRampRate(Ptr(this), rampRate, taskId);

#if 0
    double value = rampRate;
    A3200MotionSetupRampRateAccelAxis(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask(), &value);
#endif
}

/*
 * Input and Output related commands
 */
void Axis::setDigitalPin(uint64_t pin, bool state, const uint8_t taskId)
{
    uint32_t value = state;
    bool status = A3200IODigitalOutput(mController->handle(), (TASKID) taskId, 0, (AXISINDEX) mId, value);

    if(!status) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }
}

bool Axis::digitalPin(uint64_t pin, const uint8_t taskId)
{
    uint32_t value;

    bool status = A3200IODigitalInput(mController->handle(), (TASKID) taskId, pin, (AXISINDEX) mId, (DWORD *) &value);

    if(!status) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }

    return value;
}

void Axis::setAnalogPin(const uint64_t pin, const double value, const uint8_t taskId)
{
    bool status = A3200IOAnalogOutput(mController->handle(), (TASKID) taskId, pin, (AXISINDEX) mId, value);

    if(!status) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }

}

double Axis::analogPin(const uint64_t pin, const uint8_t taskId)
{
    double value;
    bool status = A3200IOAnalogInput(mController->handle(), (TASKID) taskId, pin, (AXISINDEX) mId, &value);

    if(!status) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }


    return value;
}

double Axis::maxCurrent()
{
    return this->getStatus(STATUSITEM_PeakCurrent);
}
