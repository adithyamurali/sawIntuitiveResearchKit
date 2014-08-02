#ifndef MTSINTUITIVERESEARCHKITPSMSINETASK_H
#define MTSINTUITIVERESEARCHKITPSMSINETASK_H

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robQuintic.h>

class mtsIntuitiveResearchKitBlock : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitBlock(const std::string & componentName, const double periodInSeconds);
    inline virtual ~mtsIntuitiveResearchKitBlock() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);
    const std::string & Name(void) const;

protected:
    void Init(void);
//    void SetRobotControlState(const std::string & state);
//    void GetPositionCartesian(const prmPositionCartesianGet & newSlavePosition);
//    void GetRobotControlState(const bool & newState);
    void RunPositionCartesian(void);

    bool RobotState;
    int counter;

    class RobotMaster {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionRead GetRobotState;
        mtsFunctionWrite SetPositionCartesian;
    };
    RobotMaster Generator;

    double maxTranslation;
    double totalTranslation;
    prmPositionCartesianGet currPosition;
    prmPositionCartesianSet newPosition;

private:

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitBlock);

#endif // MTSINTUITIVERESEARCHKITPSMSINETASK_H
