#include <cassert>
#include <algorithm>
#include <exception>

#include <A3200AeroBasicCommands.h>

#include <App/Controller.h>

#include "Exception.h"
#include "Controller.h"
#include "Axis.h"

#include "PSO.h"

using namespace aerotech;

PSO::PSO() : mAxis(nullptr),
             mOutputAxis(nullptr),
             mIsArmed(false),
             mIsEnabled(false)
{
    // private constructor
}

PSO::PSO(Axis::Ptr axis) : mAxis(axis),
                           mOutputAxis(nullptr)
{

}

PSO::~PSO()
{
}

void PSO::reset(bool hard, const uint8_t taskId)
{
    // PSOCONTROL X RESET
    // Sets the WindowCounter to zero and resets the state (does not delete the internal array)
    const uint32_t windowNum = 1;

    bool status;

    if(hard) {
        status = A3200PSOControl(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), PSOMODE_Reset);
    } else {
        status = A3200PSOWindowReset(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), windowNum, 2);
    }

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }

}

void PSO::setEncoderAxis(int32_t encoderId, int32_t encoderId2,
                    bool invert, uint8_t taskId)
{

    //# Configure the the x-axis encoder for single axis tracking
    //PSOTRACK X INPUT 0

    bool status = false;
    if(encoderId2 > 0) {
        status = A3200PSOTrackInputInput2(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), encoderId, encoderId2);
    } else {
        // Use only a single axis for tracking
        status = A3200PSOTrackInput(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), encoderId);
    }

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }

    // PSOWINDOW X 1 INPUT 0 INVERT - first axis for tracking

    const uint32_t windowNum = 1;

    if(!invert) {
        status = A3200PSOWindowInput(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), windowNum, encoderId);
    } else {
        status = A3200PSOWindowInputInvert(mAxis->controller()->handle(),  (TASKID) taskId, (AXISINDEX) mAxis->id(), windowNum, encoderId);
    }

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }
}

void PSO::setMode(const PSOMode mode)
{

}


bool PSO::isArmed() const
{
    return mIsArmed;
}

bool PSO::isEnabled() const
{
    return mIsEnabled;
    //return mAxis->controller()->getSingleDataSignal(mAxis, DATASIGNAL_PSOStatus);
}

uint32_t PSO::psoCounter(const uint8_t counter)
{
    DATASIGNAL signal;

    if(counter == 1) {
        signal = DATASIGNAL_PSOCounter1;
    } else if(counter == 2) {
        signal = DATASIGNAL_PSOCounter2;
    } else if(counter ==3){
        signal = DATASIGNAL_PSOCounter3;
    } else {
        throw A3200Exception("Invalid PSO counter provided) ", 0);
    }

    return mAxis->controller()->getSingleDataSignal(mAxis, signal);
}

uint32_t PSO::windowArrayIndexLocation(const uint8_t windowNumber)
{
    DATASIGNAL signal;

    if(windowNumber == 1) {
        signal = DATASIGNAL_PSOWindow1ArrayIndex;
    } else if(windowNumber == 2) {
        signal = DATASIGNAL_PSOWindow2ArrayIndex;
    } else {
        throw A3200Exception("Invalid window number provided) ", 0);
    }

    return mAxis->controller()->getSingleDataSignal(mAxis, signal);
}


void PSO::setOutput(const uint8_t pin, const uint8_t mode, uint8_t taskId)
{
    // Note - mode is zero     PSOOUTPUT X CONTROL 0 1
   bool status = A3200PSOOutputControl(mAxis->controller()->handle(),  (TASKID) taskId, (AXISINDEX) mAxis->id() , pin, mode);

   if(!status) {
       throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
   }
}

void PSO::setPulseCyclesOnly(const double totalTime, const double onTime, const double numCycles,
                       const uint8_t taskId)
{
    bool status = A3200PSOPulseCyclesOnly(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), totalTime, onTime, numCycles);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }
}


void PSO::setPulseDelayOnly(const double totalTime, const double onTime, const double delayTime,
                            const uint8_t taskId)
{
    bool status = A3200PSOPulseDelayOnly(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), totalTime, onTime, delayTime);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }
}

// PSODISTANCE X FIXED %d
void PSO::setFireDistance(const double distance, const uint8_t taskId)
{
    bool status = A3200PSODistanceFixed(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), distance);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }
}


void PSO::setWindowMask(std::vector<double> mask, EdgeMode edgeMode,  const uint32_t arrayIdx, bool hard, const uint8_t taskId)
{
    // note for HpE drive contorller, max elements in the PSO FIFO buffer is 2,097,152
    // and for the Cp Drive Controller the max elements is 262,144

    if(mask.size() > 2097152) {
        throw A3200Exception("Mask size is too large for controller", 0);
    }

    uint32_t idx = 0;

    char buffer[30];
    std::sprintf(buffer, "$global[%d]", arrayIdx);

    // A3200PSOArray
    // Define the window PSO Array to transfer
    mAxis->controller()->setGlobalVariable(arrayIdx, mask);

    // Transfer the Global ariable into the PSO Buffer
    bool status = A3200PSOArray(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), buffer, 0, mask.size());

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }

    status = false;

    if(!hard) {
        status = A3200PSOOutputPulseWindowMask(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), 0, edgeMode);
    } else {
        status = A3200PSOOutputPulseWindowMaskHard(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), 0, edgeMode);
    }

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }

    const uint32_t windowNumber = 1;
    status = A3200PSOWindowRangeArray(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), windowNumber, idx, mask.size(), 1);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    }

}



void PSO::enable(const uint8_t taskId)
{
    // A3200PSOWindowOn
    bool status = A3200PSOWindowOn(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), 1);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    } else {
        mIsEnabled = true;
    }
}

void PSO::disable(const uint8_t taskId)
{
    // A3200PSOWindowOff
    bool status = A3200PSOWindowOff(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), 1);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    } else {
        mIsEnabled = false;
    }
}

void PSO::arm(const uint8_t taskId)
{
    // A3200PSOWindowOn
    bool status = A3200PSOControl(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), PSOMODE_Arm);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    } else {
        mIsArmed = true;
    }
}

void PSO::disarm(const uint8_t taskId)
{
    // A3200PSOWindowOn
    bool status = A3200PSOControl(mAxis->controller()->handle(), (TASKID) taskId, (AXISINDEX) mAxis->id(), PSOMODE_Off);

    if(!status) {
        throw A3200Exception(mAxis->controller()->getErrorString().c_str(), mAxis->controller()->getErrorCode());
    } else {
        mIsArmed = false;
    }

}


