#include <cassert>
#include <algorithm>
#include <exception>

#include <A3200AeroBasicCommands.h>

#include "Axis.h"
#include "Controller.h"
#include "Exception.h"

using namespace aerotech;

Axis::Axis() :  mState(AxisState::DISABLED),
                mLabel(""),
                mController(nullptr)
{
    // private constructor
}


Axis::Axis(A3200Controller::Ptr controller, uint16_t axis) : mId(axis),
                                                             mState(AxisState::DISABLED),
                                                             mController(controller),
                                                             mDefaultSpeed(100)
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
    A3200MotionEnable(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask() );
}

void Axis::disable(const uint8_t taskId)
{
    A3200MotionDisable(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask());
}

void Axis::abort()
{
    A3200MotionAbort(mController->handle(), (AXISMASK) axisMask());
}

void Axis::acknowlegeFault(const uint8_t taskId)
{
    this->mController->acknowlegeFault(Ptr(this), taskId);
}


/*
 * Position and control related commands
 */
uint32_t Axis::status()
{
    return  this->getStatus(STATUSITEM_AxisStatus);
}

uint32_t Axis::driveStatus()
{
    return this->getStatus(STATUSITEM_DriveStatus);
}

bool Axis::inPosition()
{
    // AXISSTATUS_Enabling

     uint32_t driveStatus = this->getStatus(STATUSITEM_DriveStatus);

     bool statusBit = driveStatus & DRIVESTATUS_InPosition;

     return statusBit;
}

bool Axis::inMotion()
{
    // AXISSTATUS_Enabling

     uint32_t driveStatus = this->getStatus(STATUSITEM_DriveStatus);

     bool statusBit = driveStatus & DRIVESTATUS_MoveActive;

     return statusBit;
}

bool Axis::isEnabled()
{
    // AXISSTATUS_Enabling

     uint32_t driveStatus = this->getStatus(STATUSITEM_DriveStatus);

     bool statusBit = driveStatus & DRIVESTATUS_Enabled;

     return statusBit;
}

bool Axis::isEnabling()
{
    // AXISSTATUS_Enabling

     uint32_t axisStatus = this->getStatus(STATUSITEM_AxisStatus);

     bool statusBit = axisStatus & AXISSTATUS_Enabling;

     return statusBit;
}

bool Axis::isHomed()
{     
    uint32_t axisStatus = this->getStatus(STATUSITEM_AxisStatus);

    bool statusBit = axisStatus & AXISSTATUS_Homed;

    return statusBit;
}

bool Axis::isBlocked()
{
    uint32_t axisStatus = this->getStatus(STATUSITEM_AxisStatus);

    bool statusBit = axisStatus & AXISSTATUS_MotionBlocked;

    return statusBit;

}

void Axis::home(const uint8_t taskId)
{
    A3200MotionHome(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask());
}

void Axis::move(const double position, const double speed, const uint8_t taskId)
{
    if(speed > 1e-6){
        A3200MotionMoveAbs(mController->handle(), (TASKID) taskId, (AXISINDEX) mId, position, speed);
    } else {
        A3200MotionMoveAbs(mController->handle(), (TASKID) taskId, (AXISINDEX) mId, position, mDefaultSpeed);
    }
}

void Axis::moveRel(const double distance, const double speed, const uint8_t taskId)
{
    double value = distance;

    bool state;

    if(speed > 1e-6){
        state = A3200MotionMoveInc(mController->handle(), (TASKID) taskId, (AXISINDEX) mId, distance, speed);
    } else {
        state = A3200MotionMoveInc(mController->handle(), (TASKID) taskId, (AXISINDEX) mId, distance, mDefaultSpeed);
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


void Axis::wait(int32_t timeout, bool inPosition)
{
    WAITOPTION waitOption = WAITOPTION_InPosition;

    if(!inPosition)
        waitOption = WAITOPTION_MoveDone;

    A3200MotionWaitForMotionDone(mController->handle(), (AXISMASK) axisMask(), waitOption, timeout, NULL);
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
    A3200MotionSetupRampModeAxis(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask(), (RAMPMODE) rampMode);
}

void Axis::setRampRate(const double rampRate, const uint8_t taskId)
{
    double value = rampRate;
    A3200MotionSetupRampRateAccelAxis(mController->handle(), (TASKID) taskId, (AXISMASK) axisMask(), &value);
}

/*
 * Input and Output related commands
 */
void Axis::setDigitalOutputPin(uint64_t pin, bool state, const uint8_t taskId)
{
    bool status = A3200IODigitalOutputBit(mController->handle(), (TASKID) taskId, pin, (AXISINDEX) mId, state);

    if(!status) {
        throw A3200Exception(mController->getErrorString().c_str(), mController->getErrorCode());
    }
}

bool Axis::digitalOutputPin(uint64_t pin, const uint8_t taskId)
{

    uint32_t pinBit = (1 << pin);

    uint32_t digitalOutput = this->digitalOutput();

    return digitalOutput & pinBit;
}

uint32_t Axis::digitalOutput()
{
    // AXISSTATUS_Enabling
     uint32_t digitalOutput = this->getStatus(STATUSITEM_DigitalOutput);
     return digitalOutput;
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

