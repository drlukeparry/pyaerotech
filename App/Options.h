#ifndef AEROTECH_OPTIONS_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_OPTIONS_H_HEADER_HAS_BEEN_INCLUDED

#include <A3200.h>

namespace aerotech
{

enum AxisId {
    X = AXISINDEX_00,
    Y = AXISINDEX_01,
    Z = AXISINDEX_02,
    A = AXISINDEX_03,
    B = AXISINDEX_04,
    C = AXISINDEX_05,
};

enum AxisState {
    DISABLED = 0,
    ACTIVE = 1,
    HOMED = 2
};

enum RampMode {
    RATE = RAMPMODE_Rate,
    TIME = RAMPMODE_Time
};

enum RampType {
    LINEAR = RAMPTYPE_Linear,
    SCURVE = RAMPTYPE_Scurve,
    SINE =  RAMPTYPE_Sine
};

enum MotionMode {
    MOTION_ABSOLUTE = 0,
    MOTION_INCREMENTAL = 1
};

enum EdgeMode
{
    NONE = 0,
    EXIT = 1,
    ENTER = 2,
    BOTH = 3
};

}

#endif
