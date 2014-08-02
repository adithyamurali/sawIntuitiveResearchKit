/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robQuintic.h>

// temporary
#include <cisstOSAbstraction/osaStopwatch.h>

#include <fstream>
using namespace std;

class mtsIntuitiveResearchKitPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    static const size_t NumberOfJoints = 7;

    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitPSM() {file.close();}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    ofstream file;

    enum RobotStateType {
        PSM_UNINITIALIZED, /*! State when constructed */
        PSM_HOMING_POWERING, /*! Turn power on, calibrate encoders and current */
        PSM_HOMING_CALIBRATING_ARM, /*! Calibrate using pots and move to zero position for all joints except last one */
        PSM_ARM_CALIBRATED, /*! Do nothing, just wait for adapter.  Fall back state when adapter is removed. */
        PSM_ENGAGING_ADAPTER,
        PSM_ADAPTER_ENGAGED, /*! Do nothing, just wait for tool.  Fall back state when tool is removed. */ 
        PSM_ENGAGING_TOOL,
        PSM_READY,
        PSM_POSITION_CARTESIAN, /**< Go to command cartesian position */
        PSM_MANUAL /**< User manually move robot */
    };

    void Init(void);

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const RobotStateType & newState);

    /*! Homing procedure, will check the homing state and call the required method. */
    void RunHoming(void);

    /*! Homing procedure, power the robot and initial current and encoder calibration. */
    void RunHomingPower(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    /*! Engaging adapter procedure. */
    void RunEngagingAdapter(void);

    /*! Engaging tool procedure. */
    void RunEngagingTool(void);

    /*! Cartesian state. */
    void RunPositionCartesian(void);

    /*! Wrapper to convert vector of 7 values to prmPositionJointSet and send to PID */
    void SetPositionJointLocal(const vctDoubleVec & newPosition);

    void EventHandlerAdapter(const prmEventButton & button);
    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);
    void EventHandlerSUJClutch(const prmEventButton & button);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetOpenAngle(const double & openAngle);
    void SetRobotControlState(const std::string & state);

    struct {
        mtsFunctionWrite Enable;
        mtsFunctionRead GetPositionJoint;
        mtsFunctionRead GetPositionJointDesired;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetCheckJointLimit;
    } PID;

    // Required interface
    struct InterfaceRobotTorque {
        //! Enable Robot Power
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionVoid BiasEncoder;
        mtsFunctionWrite SetMotorCurrent;
    } RobotIO;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Adapter;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Tool;

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } SUJClutch;

    // Functions for events
    struct {
        mtsFunctionWrite RobotStatusMsg;
        mtsFunctionWrite RobotErrorMsg;
        mtsFunctionWrite ManipClutch;
        RobotStateType ManipClutchPreviousState;
        mtsFunctionWrite SUJClutch;
    } EventTriggers;

    // ZC: cache Cartesian Goal posiiton
    prmPositionCartesianSet CartesianGoalSet;
    bool IsCartesianGoalSet;

    prmPositionCartesianGet CartesianCurrentParam;
    vctFrm4x4 CartesianCurrent;
    vctFrm4x4 CartesianPrevious;
    prmPositionJointGet JointCurrentParam;
    vctDoubleVec JointCurrent;
    prmPositionJointSet JointDesiredParam;
    prmPositionJointGet JointSimulationParam;
    vctDoubleVec JointDesired;
    robManipulator Manipulator;
    vctFrm4x4 Frame6to7;
    vctFrm4x4 Frame6to7Inverse;

    vctFrm4x4 CartesianPositionFrm;
    double DesiredOpenAngle;
    RobotStateType RobotState;

    struct {
        robQuintic Quintic;
        vctDoubleVec Start;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Goal;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        vctDoubleVec Zero;
    } JointTrajectory;

    // Home Action
    double HomingTimer;
    bool HomingPowerRequested;
    bool HomingCalibrateArmStarted;
    bool EngagingAdapterStarted;
    bool EngagingToolStarted;
    bool autonomous;

    // temporary
    osaStopwatch EngagingStopwatch;
    vctDoubleVec EngagingJointSet;

    int Counter;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
