#ifndef AEROTECH_OPTIONS_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_OPTIONS_H_HEADER_HAS_BEEN_INCLUDED

#include <A3200.h>

namespace aerotech
{

enum AxisStatus {

    /// \brief The axis is homed.
    HOME = AXISSTATUS_Homed,
    /// \brief The axis is performing coordinated (LINEAR, CW, CCW, BEZIER), RAPID, or PVT motion.
    PROFILING = AXISSTATUS_Profiling,
    /// \brief The controller finished waiting for motion on this axis to complete. The behavior of this bit depends on the selected wait mode. When in WAIT MODE MOVEDONE, this bit will mimic the Move Done bit, but when in WAIT MODE INPOS, this bit will not be active until both the Move Done bit and the In Position bit are both active.
    WAITDONE = AXISSTATUS_WaitDone,
    /// \brief Motion on the axis is controlled from the SMC.
    COMMAND_VALID = AXISSTATUS_CommandValid,
    /// \brief The axis is currently homing.
    HOMING = AXISSTATUS_Homing,
    /// \brief The axis is currently enabling.
    ENABLING = AXISSTATUS_Enabling,
    /// \brief This bit represents internal status.
    JOG_GENERATING = AXISSTATUS_JogGenerating,
    /// \brief The axis is performing asynchronous motion (MOVEINC, MOVEABS, FREERUN).
    JOGGING = AXISSTATUS_Jogging,
    /// \brief The SMC sent a command to the drive that will cause the drive to take control of the motion, but the drive has not yet done so.
    DRIVE_PENDING = AXISSTATUS_DrivePending,
    /// \brief The SMC sent an abort command to the drive, but the drive has not yet started the abort.
    DRIVE_ABORT_PENDING = AXISSTATUS_DriveAbortPending,
    /// \brief Trajectory filtering is enabled for this axis using either the TrajectoryIIRFilter or TrajectoryFIRFilter parameters.
    TRAJECTORY_FILTERING = AXISSTATUS_TrajectoryFiltering,
    /// \brief Infinite Field of View (IFOV) is enabled for this axis. Enable IFOV by issuing the IFOV ON command. Disable IFOV by issuing the IFOV OFF command.
    IFOV_ENABLED = AXISSTATUS_IFOVEnabled,
    /// \brief A physical drive is associated with this axis. Axes with no drive attached will clear this bit and operate as virtual axes.
    NOT_VIRTUAL = AXISSTATUS_NotVirtual,
    /// \brief The specified 1D calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
    CALIBRATION_ENABLED_1D = AXISSTATUS_CalibrationEnabled1D,
    /// \brief The specified 2D calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
     CALIBRATION_ENABLED_2D = AXISSTATUS_CalibrationEnabled2D,
    /// \brief The axis is currently performing motion under master/slave control (gearing, camming, or handwheel).
    MASTER_SLAVE_CONTROL = AXISSTATUS_MasterSlaveControl,
    /// \brief The axis is currently performing motion under control of the JOYSTICK command.
    JOYSTICK_CONTROL = AXISSTATUS_JoystickControl,
    /// \brief Backlash compensation is enabled for this axis using the BacklashDistance parameter or the BACKLASH ON command.
    BACKLASH_ACTIVE = AXISSTATUS_BacklashActive,
    /// \brief A Gain Mapping table was specified for this axis.
    GAIN_MAPPING_ENABLED = AXISSTATUS_GainMappingEnabled,
    /// \brief The axis is considered to be stable as configured by the Stability0Threshold and Stability0Time parameters.
    STABILITY0 = AXISSTATUS_Stability0 ,
    /// \brief Motion on this axis is being prevented by the BLOCKMOTION command.
    MOTION_BLOCKED = AXISSTATUS_MotionBlocked,
    /// \brief Motion on this axis is done, meaning that the velocity command reached zero.
    MOVE_DONE = AXISSTATUS_MoveDone,
    /// \brief Motion on this axis is being clamped due to a software limit clamp or safe zone. Refer to the SoftwareLimitSetup parameter, and the Safe zone overview.
    MOTION_CLAMPED = AXISSTATUS_MotionClamped,
    /// \brief This axis is part of a gantry pair and the gantry is correctly aligned. This bit will not be active until the gantry axes have been homed.
    GANTRY_ALIGNED = AXISSTATUS_GantryAligned,
    /// \brief The axis is currently performing gantry realignment motion.
    GANTRY_REALIGNING = AXISSTATUS_GantryRealigning,
    /// \brief The axis is considered to be stable as configured by the Stability1Threshold and Stability1Time parameters.
    STABILITY1 = AXISSTATUS_Stability1,
    /// \brief The ThermoComp feature is currently turned on.
    THERMCOMP_ENABLED = AXISSTATUS_ThermoCompEnabled
};

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
