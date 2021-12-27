#ifndef AEROTECH_CONTROLLER_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_CONTROLLER_H_HEADER_HAS_BEEN_INCLUDED

#include "AEROTECH_Export.h"

#include <A3200.h>

#include <cstdint>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include "Options.h"
#include "Axis.h"

namespace aerotech
{

class AEROTECH_EXPORT A3200Controller
{


public:

    typedef std::shared_ptr<A3200Controller> Ptr;

    A3200Controller();
    ~A3200Controller();

public:

    std::string getErrorString() const;
    ErrorCode getErrorCode() const;

    void initialise();
    void connect();
    void reset();
    void dwell(const double time);

    A3200Handle handle() { return mHandle; }

    void disconnect();
    bool isConnected() const { return mIsConnected; }

    void runScript(const std::string &filename, const uint8_t taskId = 1);

    double runCommand(const std::string &command, const uint8_t taskId = 1);
    void stopProgram(const uint32_t timeout, const uint8_t taskId);

    /* Queue related commands */
    void startTaskQueue(const uint8_t taskId = 1);
    void blockUntilQueueComplete(const uint32_t pollingTime = 100, const uint8_t taskId = 1);
    void endTaskQueue(bool hard = false, const uint32_t timeout = 1000, const uint8_t taskId = 1);
    uint32_t queueCount();
    uint32_t maxQueueSize();


    bool isProgramRunning();

    void criticalStart(double time = -1.0);
    void criticalEnd();

    double getDataSignal();

    double getAxisStatus(Axis::Ptr axis, const uint32_t item);
    double getTaskStatus(const uint8_t taskId, const uint32_t item);

    void setGlobalVariable(const uint32_t idx, std::vector<double> &value);
    void setGlobalVariable(const uint32_t idx, const double value);

    void setVariable(const std::string &str, std::vector<double> &value);
    void setVariable(const std::string &str, const double value);


    Eigen::MatrixXd getDataSignal(Axis::Ptr axis,  std::vector<uint32_t> dataSignals, uint32_t numPoints, uint32_t samplePeriod = 1);
    double getSingleDataSignal(Axis::Ptr axis, uint32_t dataSignal);


public:

    void acknowledgeAll(const uint8_t taskId = 1);

    void enable(const std::vector<Axis::Ptr> &axes, const uint8_t taskId = 1);
    void enable(Axis::Ptr axis, const uint8_t taskId = 1);

    void disable(const std::vector<Axis::Ptr> &axes, const uint8_t taskId = 1);
    void disable(Axis::Ptr axis, const uint8_t taskId = 1);

    void abort(const std::vector<Axis::Ptr> &axes);
    void abort(Axis::Ptr axis);

    void acknowlegeFault(const std::vector<Axis::Ptr> &axes, const uint8_t taskId = 1);
    void acknowlegeFault(Axis::Ptr axis, const uint8_t taskId = 1);

    void home(const std::vector<Axis::Ptr> &axes, const uint8_t taskId = 1);
    void home(Axis::Ptr axis, const uint8_t taskId = 1);

    void move(const std::vector<Axis::Ptr> &axes,  std::vector<double> distance, const double speed = -1.0, const uint8_t taskId = 1);
    void move(Axis::Ptr axis, const double distance, const double speed = -1.0, const uint8_t taskId = 1);

    void wait(const std::vector<Axis::Ptr> &axes, uint64_t timeout, bool inPosition = false);
    void wait(Axis::Ptr axis, uint64_t timeout, bool inPosition = false);

    std::vector<double> position(const std::vector<Axis::Ptr> &axes) const;
    double position(Axis::Ptr axis) const;

    std::vector<double> velocity(const std::vector<Axis::Ptr> &axes, bool average = false) const;
    double velocity(Axis::Ptr axis, bool average = false) const;


    // Set configuration
    void setRampRate(const std::vector<Axis::Ptr> &axes, const double rampRate, const uint8_t taskId = 1);
    void setRampRate(Axis::Ptr axis, const double rampRate, const uint8_t taskId = 1);

    void setRampMode(const std::vector<Axis::Ptr> &axes, const RampMode rampMode, const uint8_t taskId = 1);
    void setRampMode(Axis::Ptr axis, const RampMode rampMode, const uint8_t taskId = 1);

    void setMotionMode(const MotionMode mode, const uint8_t taskId = 1);
    void setDefaultSpeed(const double speed);


    /*
     * Axis methods
     */
    AXISMASK getAxisMask(const std::vector<Axis::Ptr> &axes);
    AXISMASK getAxisMask(Axis::Ptr axis);

    std::vector<Axis::Ptr> & axis() { return mAxis; }
    void addAxis(Axis::Ptr axis);
    void clearAxes();


protected:    
    std::vector<Axis::Ptr> mAxis;
    double mDefaultSpeed;

private:
    bool mIsConnected;
    MotionMode mMotionMode;

    A3200Handle mHandle;
    static int numControllers;
};

} // End of Namespace slm


#endif // AEROTECH_CONTROLLER_H_HEADER_HAS_BEEN_INCLUDED
