#include <cassert>
#include <algorithm>
#include <exception>

#include <iostream>
#include <fstream>

#include <filesystem/fwd.h>
#include <filesystem/resolver.h>
#include <filesystem/path.h>
#include <fmt/format.h>

#include "Controller.h"
#include "Exception.h"

using namespace aerotech;

int A3200Controller::numControllers = 0;


/*
 * Contoller Methods
 */

A3200Controller::A3200Controller() : mIsConnected(false),
                                     mHandle(nullptr),
                                     mDefaultSpeed(1000),
                                     mMotionMode(MOTION_ABSOLUTE)
{

}

A3200Controller::~A3200Controller()
{
    this->disconnect();
}

std::string A3200Controller::getErrorString() const
{
    char data[1024];
    A3200GetLastErrorString(data, 1024);

    return std::string(data);
}

ErrorCode A3200Controller::getErrorCode() const
{
   return A3200GetLastError().Code;
}


void A3200Controller::initialise()
{
    // Set default motion to absolute
    this->setMotionMode(MOTION_ABSOLUTE);
}

void A3200Controller::startTaskQueue(const uint8_t taskId)
{
    // Put task into Queue mode.
    bool status = A3200ProgramInitializeQueue(mHandle, (TASKID) taskId);


    if(!status) {
        throw A3200Exception(this->getErrorString().c_str(), this->getErrorCode());
    }

}

void A3200Controller::blockUntilQueueComplete(const uint32_t pollingTime, const uint8_t taskId)
{
    int32_t queueLineCount = 1;

    while(queueLineCount) {
            // If there are still commands to execute sleep, and then check again on how many
            // commands are left.
            Sleep(pollingTime);
            queueLineCount = this->queueCount();
            //(handle, TASKID_01, STATUSITEM_QueueLineCount, 0, &queueLineCount);
    }

}

uint32_t A3200Controller::queueCount()
{
    const uint8_t taskId = 1;

    double queueCount = this->getTaskStatus(taskId, STATUSITEM_QueueLineCount);

    return queueCount;
}

uint32_t A3200Controller::maxQueueSize()
{
    const uint8_t taskId = 1;

    double queueCount = this->getTaskStatus(taskId, STATUSITEM_QueueLineCapacity);

    return queueCount;
}

void A3200Controller::endTaskQueue(bool hard, const uint32_t timeOut, const uint8_t taskId)
{
    bool status = false;

    if(hard) {
        status = A3200ProgramStop(mHandle, (TASKID) taskId);
    } else {
        status = A3200ProgramStopAndWait(mHandle, (TASKID) taskId, timeOut);
    }

    if(!status) {
        throw A3200Exception(this->getErrorString().c_str(), this->getErrorCode());
    }
}

double A3200Controller::getTaskStatus(const uint8_t taskId, const uint32_t item)
{
    double value;

    bool state = A3200StatusGetItem(mHandle, (TASKID) taskId, (STATUSITEM) item, 0, &value);

    if(!state) {
        throw A3200Exception(this->getErrorString().c_str(), this->getErrorCode());
    }

    return value;
}

double A3200Controller::getAxisStatus(Axis::Ptr axis, const uint32_t item)
{
    double value;
    bool state = A3200StatusGetItem(mHandle, (AXISINDEX) axis->id(),  (STATUSITEM) item, 0, &value);

    if(!state) {
        throw A3200Exception(this->getErrorString().c_str(), this->getErrorCode());
    }

    return value;
}

double A3200Controller::getSingleDataSignal(Axis::Ptr axis, uint32_t dataSignal)
{
    A3200DataCollectConfigHandle dcchandle = NULL;
    A3200_DATACOLLECTION_STATUS status;

    double data;

    A3200DataCollectionConfigCreate(mHandle, &dcchandle);

    // Remove existing signals from the collection
    A3200DataCollectionConfigRemoveSignalAll(dcchandle);

    //DATASIGNAL_PositionCommand
    A3200DataCollectionConfigAddSignal(dcchandle, (DATASIGNAL) dataSignal, (AXISINDEX) axis->id(), 0);

    // Collect one sample across (1ms)
    A3200DataCollectionConfigSetPeriod(dcchandle, 1);

    // the number of samples to collect
    A3200DataCollectionConfigSetSamples(dcchandle, 1);
    A3200DataCollectionStart(mHandle, dcchandle);

    // Collect a single sample point
    A3200DataCollectionGetStatus(mHandle, &status);

    // collect all the data points requested
    // Signal count must match parameters
    A3200DataCollectionDataRetrieve(mHandle, 1, 1, &data);

    // TODO: add processing of data here

    A3200DataCollectionStop(mHandle);
    A3200DataCollectionConfigFree(dcchandle);

    return data;


}


Eigen::MatrixXd A3200Controller::getDataSignal(Axis::Ptr axis, std::vector<uint32_t> dataSignals, uint32_t numPoints, uint32_t samplePeriod)
{
    A3200DataCollectConfigHandle dcchandle = NULL;
    A3200_DATACOLLECTION_STATUS status;

    const uint32_t numSignals = dataSignals.size();

    Eigen::MatrixXd data;
    data.resize(numPoints, numSignals);

    //double buffer[2][3000];
    int pointsRetrieved = 0;

    A3200DataCollectionConfigCreate(mHandle, &dcchandle);

    // Remove existing signals from the collection
    A3200DataCollectionConfigRemoveSignalAll(dcchandle);

    //DATASIGNAL_PositionCommand
    //
    for(auto signal : dataSignals)
        A3200DataCollectionConfigAddSignal(dcchandle, (DATASIGNAL) signal, (AXISINDEX) axis->id(), 0);

    A3200DataCollectionConfigSetPeriod(dcchandle, samplePeriod);

    // the number of samples to collect
    A3200DataCollectionConfigSetSamples(dcchandle, numPoints);
    A3200DataCollectionStart(mHandle, dcchandle);

    // Note: this is a blocking function, so it has very limited use in pratical applications
    // wait until every data point is collected
    while(pointsRetrieved < numPoints) {
            A3200DataCollectionGetStatus(mHandle, &status);
            pointsRetrieved = status.samplePointCollected;
    }

    // collect all the data points requested
    // Signal count must match parameters
    A3200DataCollectionDataRetrieve(mHandle, numSignals, numPoints, (double *) data.data());

    // TODO: add processing of data here

    A3200DataCollectionStop(mHandle);
    A3200DataCollectionConfigFree(dcchandle);

    return data;


}

void A3200Controller::connect()
{
    if(A3200Controller::numControllers > 0) {
        return;
    }

    if(!A3200Connect(&mHandle)) {
        std::string error = this->getErrorString();
    } else {
        // A3220 Controller is connected
        mIsConnected = true;
        A3200Controller::numControllers += 1;

        // Initialise the controller with default options
        this->initialise();
    }

}

void A3200Controller::reset()
{
    A3200Reset(mHandle);
}


void A3200Controller::disconnect()
{
    if(!A3200Disconnect(mHandle)) {
        std::string error = this->getErrorString();
    } else {
        A3200Controller::numControllers =- 1;
        mIsConnected = false;
    }
}


void A3200Controller::runScript(const std::string &filename, const uint8_t taskId)
{
    bool state = A3200ProgramRun(mHandle, (TASKID) taskId, filename.c_str());

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}


double A3200Controller::runCommand(const std::string &command, const bool returnValue, const uint8_t taskId)
{
    // TODO IMPLEMENT
    double value = 1.0;

    bool state = false;

    if(returnValue)
        state = A3200CommandExecute(mHandle, (TASKID) taskId, command.c_str(), &value);
    else
        state = A3200CommandExecute(mHandle, (TASKID) taskId, command.c_str(), nullptr);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

    return value;
}


void A3200Controller::stopProgram(const uint32_t timeout, const uint8_t taskId)
{
    bool state;

    if(timeout > 0) {
        state = A3200ProgramStopAndWait(mHandle, (TASKID) taskId, timeout );
    } else {
        state = A3200ProgramStop(mHandle, (TASKID) taskId);
    }

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}


bool A3200Controller::isProgramRunning()
{
    //double value;
    //A3200StatusGetItem(handle, TASKID_01, STATUSITEM_, 0, &value);
    return false;
}

void A3200Controller::dwell(const double time)
{
    double result;
    char buffer[50];

    std::string cmd = fmt::format("DWELL {:.3f} \n", time);
    std::sprintf(buffer, "DWELL %.3f \n", time);

    bool state = A3200CommandExecute(mHandle, TASKID_Library, cmd.c_str(), nullptr);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::criticalStart(double time)
{
    double result;
    char buffer[50];

    std::string cmd;

    if(time > 1e-6) {
        cmd = fmt::format("CRITICAL START {:.3f} \n", time);
        std::sprintf(buffer, "CRITICAL START %.3f \n", time);
    } else {
        cmd = fmt::format("CRITICAL START \n");
        std::sprintf(buffer, "CRITICAL START\n");
    }


    std::string s = fmt::format("The answer is {} \n", 42);

    bool state = A3200CommandExecute(mHandle, TASKID_Library, cmd.c_str(), nullptr);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

}

void A3200Controller::criticalEnd()
{
    double result;
    char buffer[50];

    std::sprintf(buffer, "CRITICAL END \n", time);
    std::string cmd = fmt::format("CRITICAL END \n");

    bool state = A3200CommandExecute(mHandle, TASKID_Library, cmd.c_str(), nullptr);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

}


void A3200Controller::acknowledgeAll(const uint8_t taskId)
{
    bool state = A3200AcknowledgeAll(mHandle, (TASKID) taskId);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}


void A3200Controller::enable(const std::vector<Axis::Ptr> &axes, const uint8_t taskId)
{
    if(!mIsConnected)
        return;

    bool state = A3200MotionEnable(mHandle, (TASKID) taskId,  getAxisMask(axes));

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::enable(Axis::Ptr axis, const uint8_t taskId)
{
    this->enable(std::vector<Axis::Ptr> {axis}, taskId);
}

void A3200Controller::disable(const std::vector<Axis::Ptr> &axes, const uint8_t taskId)
{
    bool state = A3200MotionDisable(mHandle, (TASKID) taskId,  getAxisMask(axes));

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::disable(Axis::Ptr axis, const uint8_t taskId)
{
    this->disable(std::vector<Axis::Ptr> {axis}, taskId);
}

void A3200Controller::abort(const std::vector<Axis::Ptr> &axes)
{
    bool state = A3200MotionAbort(mHandle, getAxisMask(axes));

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::abort(Axis::Ptr axis)
{
   this->abort(std::vector<Axis::Ptr> {axis});
}

void A3200Controller::acknowlegeFault(const std::vector<Axis::Ptr> &axes, const uint8_t taskId)
{
    bool state = A3200MotionFaultAck(mHandle, (TASKID) taskId, getAxisMask(axes));

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::acknowlegeFault(Axis::Ptr axis, const uint8_t taskId)
{
    this->acknowlegeFault(std::vector<Axis::Ptr> {axis}, taskId);
}

void A3200Controller::home(const std::vector<Axis::Ptr> &axes, const uint8_t taskId)
{
    bool state = A3200MotionHome(mHandle, (TASKID) taskId, getAxisMask(axes));

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::home(Axis::Ptr axis, const uint8_t taskId)
{
    this->home(std::vector<Axis::Ptr> {axis}, taskId);
}

void A3200Controller::move(const std::vector<Axis::Ptr> &axes, std::vector<double> distance, const double speed, const uint8_t taskId)
{
    if(!mIsConnected)
        throw A3200Exception("Controller is not connected or initialised", 0);

    if(axes.size() != distance.size())
        throw A3200Exception("Number of axes and position components must match", 0);

    bool state;

    if(speed > 1e-6){
        state = A3200MotionLinearVelocity(mHandle, (TASKID) taskId, getAxisMask(axes), distance.data(), speed);
    } else {
        state = A3200MotionLinear(mHandle, (TASKID) taskId, getAxisMask(axes) , distance.data());
    }

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::move(Axis::Ptr axis, const double distance, const double speed, const uint8_t taskId)
{
    this->move(std::vector<Axis::Ptr>{axis}, std::vector<double>{distance}, speed, taskId);
}

void A3200Controller::setDefaultSpeed(const double speed)
{
    // todo implement
    mDefaultSpeed = speed;
}

void A3200Controller::wait(const std::vector<Axis::Ptr> &axes, uint64_t timeout, bool inPosition)
{

    WAITOPTION waitOption = WAITOPTION_InPosition;

    if(!inPosition)
        waitOption = WAITOPTION_MoveDone;

    bool state = A3200MotionWaitForMotionDone(mHandle, getAxisMask(axes), waitOption, timeout, NULL);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

}

void A3200Controller::wait(Axis::Ptr axis, uint64_t timeout, bool inPosition)
{
    this->wait(std::vector<Axis::Ptr>{axis}, timeout, inPosition);
}


void A3200Controller::setMotionMode(const MotionMode mode, const uint8_t taskId)
{
    if(!mIsConnected)
        return;

    bool state;
    if(mode == MOTION_ABSOLUTE) {
        state = A3200MotionSetupAbsolute(mHandle, (TASKID) taskId);
    } else {
        state = A3200MotionSetupIncremental(mHandle, (TASKID) taskId);
    }

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

    mMotionMode = mode;

}

void A3200Controller::setRampMode(const std::vector<Axis::Ptr> &axes, const RampMode rampMode, const uint8_t taskId)
{
    if(!mIsConnected)
        return;

    bool state = A3200MotionSetupRampModeAxis(mHandle, (TASKID) taskId, getAxisMask(axes), (RAMPMODE) rampMode);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

}

void A3200Controller::setRampMode(Axis::Ptr axis, const RampMode rampMode, const uint8_t taskId)
{
   this->setRampMode(std::vector<Axis::Ptr>{axis}, rampMode, taskId);
}

void A3200Controller::setRampRate(const std::vector<Axis::Ptr> &axes, const double rampRate, const uint8_t taskId)
{
    if(!mIsConnected)
        return;

    double value = rampRate; // copy required due to const qualifier
    bool state = A3200MotionSetupRampRateAccelAxis(mHandle, (TASKID) taskId, getAxisMask(axes), &value);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::setRampRate(Axis::Ptr axis, const double rampRate, const uint8_t taskId)
{
    this->setRampRate(std::vector<Axis::Ptr>{axis}, rampRate, taskId);
}

std::vector<double> A3200Controller::position(const std::vector<Axis::Ptr> &axes) const
{
    // we want information for axis #0 and Task #1

    if(!mIsConnected)
        return std::vector<double>();

    std::vector<uint16_t> axisList;
    axisList.reserve(axes.size());

    std::vector<STATUSITEM> statusList;
    std::vector<double> valueList;
    std::vector<uint32_t> itemExtraList;

    for(auto axis : axes) {
        axisList.push_back(axis->id());
        statusList.push_back(STATUSITEM_PositionFeedback);
        valueList.push_back(0.0);
        itemExtraList.push_back(0);
    }

    /*
    WORD itemAxisTaskIndexArray[] = { AXISINDEX_00, AXISINDEX_00, TASKID_01 };
    // we want position command, position feedback, and task mode
    STATUSITEM itemCodeArray[] = { STATUSITEM_PositionCommand, STATUSITEM_PositionFeedback, STATUSITEM_TaskMode };
    // extra items usually do not matter, but for some items they do; we want only a few bits of task mode
    DWORD itemExtrasArray[] = { 0, 0, TASKMODE_Absolute|TASKMODE_Minutes|TASKMODE_Secondary };
    DOUBLE itemValuesArray[3];
    // get all 3 items together
    */
    bool state = A3200StatusGetItems(mHandle, axisList.size(), axisList.data(), statusList.data(), (unsigned long *) itemExtraList.data(), valueList.data());

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

    return valueList;
}


double A3200Controller::position(Axis::Ptr axis) const
{

    std::vector<double> out = this->position(std::vector<Axis::Ptr>{axis});

    if(out.size() == 0) {
        // todo some error check required
        return 0.0;
    } else {
        return out[0];
    }
}

std::vector<double> A3200Controller::velocity(const std::vector<Axis::Ptr> &axes, bool average) const
{
    // we want information for axis #0 and Task #1


    std::vector<uint16_t> axisList;
    axisList.reserve(axes.size());

    std::vector<STATUSITEM> statusList;
    std::vector<double> valueList;
    std::vector<uint32_t> itemExtraList;

    for(auto axis : axes) {
        axisList.push_back(axis->id());
        statusList.push_back((average) ? STATUSITEM_VelocityFeedbackAverage : STATUSITEM_VelocityFeedback);
        valueList.push_back(0.0);
        itemExtraList.push_back(0);
    }

    /*
    WORD itemAxisTaskIndexArray[] = { AXISINDEX_00, AXISINDEX_00, TASKID_01 };
    // we want position command, position feedback, and task mode
    STATUSITEM itemCodeArray[] = { STATUSITEM_PositionCommand, STATUSITEM_PositionFeedback, STATUSITEM_TaskMode };
    // extra items usually do not matter, but for some items they do; we want only a few bits of task mode
    DWORD itemExtrasArray[] = { 0, 0, TASKMODE_Absolute|TASKMODE_Minutes|TASKMODE_Secondary };
    DOUBLE itemValuesArray[3];
    // get all 3 items together
    */
    bool state = A3200StatusGetItems(mHandle, axisList.size(), axisList.data(), statusList.data(), (unsigned long *) itemExtraList.data(), valueList.data());

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

    return valueList;
}

double A3200Controller::velocity(Axis::Ptr axis, bool average) const
{
    std::vector<double> out = this->velocity(std::vector<Axis::Ptr>{axis}, average);

    if(out.size() == 0) {
        // todo some error check required
        return 0.0;
    } else {
        return out[0];
    }
}

AXISMASK A3200Controller::getAxisMask(const std::vector<Axis::Ptr> &axes)
{
    uint32_t mask = 0;
    for(auto axis : axes)
        mask |= axis->axisMask();

    return (AXISMASK) mask;
}

AXISMASK A3200Controller::getAxisMask(Axis::Ptr axis)
{
    return (AXISMASK) axis->axisMask();
}


void A3200Controller::addAxis(Axis::Ptr axis)
{
    mAxis.push_back(axis);
}

void A3200Controller::clearAxes()
{
    mAxis.clear();
}


void A3200Controller::setGlobalVariable(const uint32_t idx, const double value)
{
    bool state = A3200VariableSetGlobalDouble(mHandle, idx, value);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }

}

void A3200Controller::setGlobalVariable(const uint32_t idx, std::vector<double> &value)
{
    bool state = A3200VariableSetGlobalDoubles(mHandle, idx, value.data(), value.size());

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}

void A3200Controller::setVariable(const std::string &str, const double value, uint8_t taskId)
{
    bool state = A3200VariableSetValueByName(mHandle, (TASKID) taskId, str.c_str(), value);

    if(!state) {
        throw A3200Exception(getErrorString().c_str(), getErrorCode());
    }
}
