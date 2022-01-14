#ifndef AEROTECH_AXIS_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_AXIS_H_HEADER_HAS_BEEN_INCLUDED

#include "AEROTECH_Export.h"

#include <A3200.h>

#include <cstdint>
#include <vector>
#include <memory>


#include <Eigen/Dense>

#include "Options.h"


namespace aerotech
{
    // Forward declarations
    class A3200Controller;
}

namespace aerotech
{

/*
 * Below are assumed axis defaults
 */



class AEROTECH_EXPORT Axis
{

public:

    typedef std::shared_ptr<Axis> Ptr;

    Axis( std::shared_ptr<A3200Controller> controller, uint16_t axis);

    ~Axis();
    
public:

    //State state() const {return mState; };

    void enable(const uint8_t taskId = 1);
    void disable(const uint8_t taskId = 1);
    void abort();
    void acknowlegeFault(const uint8_t taskId = 1);

    double getStatus(const STATUSITEM &item);

    /*
     * Position and control related commands
     */

    uint8_t id() const { return mId; }
    uint32_t axisMask() const;
    
    uint32_t status();
    uint32_t driveStatus();

    bool isEnabled();
    bool isEnabling();
    bool inMotion();
    bool isHomed();
    bool inPosition();
    bool isBlocked();

    void setLabel(const std::string &label) { mLabel = label; }
    const std::string & label() const { return mLabel; }

    void home(const uint8_t taskId = 1);

    void setRampMode(RampMode rampMode, const uint8_t taskId = 1);
    void setRampRate(const double rampRate, const uint8_t taskId = 1);

    void setDefaultSpeed(double speed);
    double defaultSpeed() { return mDefaultSpeed; }

    void move(const double position, const double speed = -1.0, const uint8_t taskId = 1);
    void moveRel(const double distance, const double speed = -1.0, const uint8_t taskId = 1);

    void wait(uint64_t timeout = -1, bool inPosition = false);

    double position();
    double velocity(const bool average = false);
    double current(const bool average = false);

    /*
     * Input and Output related commands
     */
    void setDigitalOutputPin(uint64_t pin, bool state, const uint8_t taskId = 1);
    bool digitalOutputPin(uint64_t pin, const uint8_t taskId = 1);
    uint32_t digitalOutput();

    void setAnalogPin(const uint64_t pin, const double value, const uint8_t taskId = 1);
    double analogPin(const uint64_t pin, const uint8_t taskId = 1);

    std::shared_ptr<A3200Controller> controller() { return mController; }

public:
    double maxCurrent();

protected:
    uint8_t mId;
    uint64_t mAxisMask;
    
    double mDefaultSpeed;
    AxisState mState;
    std::string mLabel;

    std::shared_ptr<A3200Controller> mController;
    
private:
    // Default construtor should be private only
    Axis();
    

};

} // end of namespace aerotech

#endif // AEROTECH_AXIS_H_HEADER_HAS_BEEN_INCLUDED
