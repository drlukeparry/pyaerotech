#ifndef AEROTECH_PSO_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_PSO_H_HEADER_HAS_BEEN_INCLUDED

#include "AEROTECH_Export.h"
#include <cstdint>
#include <vector>
#include <memory>


#include <Eigen/Dense>

#include "Options.h"


namespace aerotech
{
    // Forward declarations
    class A3200Controller;
    class Axis;
}

namespace aerotech
{

/*
 * Below are assumed axis defaults
 */


enum PSOMode
{
    MASK = 1,
    DISTANCE = 2
};

class AEROTECH_EXPORT PSO
{
    
public:
    
    typedef std::shared_ptr<PSO> Ptr;
    
    PSO(std::shared_ptr<Axis> axis);
    
    ~PSO();
    
public:
    
    //# Configure the the x-axis encoder for single axis tracking
    //PSOTRACK X INPUT 0
    // PSOWINDOW X 1 INPUT 0 INVERT - first axis for tracking
    void setEncoderAxis(int32_t encoderId, int32_t encoderId2 = -1,
                        bool invert = false, uint8_t taskId = 1);
    void setMode(const PSOMode mode);
    
    // Note - mode is zero     PSOOUTPUT X CONTROL 0 1
    void setOutput(const uint8_t pin = 0, const uint8_t mode = 1, uint8_t taskId = 1);
    
    void setPulseCyclesOnly(const double totalTime, const double onTime, const double cycles,
                           const uint8_t taskId = 1);
    
    
    void setPulseDelayOnly(const double totalTime, const double onTime, const double delayTime,
                           const uint8_t taskId = 1);
    
    // PSODISTANCE X FIXED %d
    void setFireDistance(const double distance, const uint8_t taskId = 1);
    void clearFireDistance(const uint8_t taskId = 1);
    void setFireContiniously(const uint8_t taskId = 1);
    
    void setWindowMask(std::vector<double> mask, EdgeMode edgeMode,  const uint32_t arrayIdx, bool hard = false,  const uint8_t taskId = 1);

    bool isArmed() const;
    bool isEnabled() const;
    uint32_t psoCounter(const uint8_t windowNumber);
    uint32_t windowArrayIndexLocation(const uint8_t windowNumber = 1);

    void arm(const uint8_t taskId = 1); // A3200PSOWindowOn
    void enableWindow(const uint8_t taskId = 1); // A3200PSOWindowOn
    void disableWindow(const uint8_t taskId = 1); // A3200PSOWindowOff
    void enable(const uint8_t taskId = 1); // A3200PSOWindowOn
    void disable(const uint8_t taskId = 1); // A3200PSOWindowOff
    void reset(bool hard = true, const uint8_t taskId = 1);
    void disarm(const uint8_t taskId = 1);
    
    std::shared_ptr<Axis> axis() const { return mAxis; }
    std::shared_ptr<Axis> outputAxis() const { return mOutputAxis; }

protected:
    std::shared_ptr<Axis> mAxis;
    std::shared_ptr<Axis> mOutputAxis;
    
private:
    bool mIsArmed;
    bool mIsEnabled;
    PSO();
};

} // end of namespace aerotech

#endif // AEROTECH_PSO_H_HEADER_HAS_BEEN_INCLUDED
