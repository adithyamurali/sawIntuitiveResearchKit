// system include
#include <time.h>
#include <math.h>
// cisst
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitBlock.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <iostream>
using namespace std;

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitBlock)

mtsIntuitiveResearchKitBlock::mtsIntuitiveResearchKitBlock(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

void mtsIntuitiveResearchKitBlock::Init(void)
{
    counter = 0;
    totalTranslation = 0;
    maxTranslation = 0.0100;
    counter = 0;
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("slave");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetPositionCartesian", Generator.SetPositionCartesian);
        interfaceRequired->AddFunction("GetPositionCartesian", Generator.GetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotState", Generator.GetRobotState);
    }
}

void mtsIntuitiveResearchKitBlock::RunPositionCartesian(void)
{
    if (totalTranslation < maxTranslation) {
        counter++;
        vctQuatRot3 quatCurr(currPosition.Position().Rotation(), VCT_NORMALIZE);
        newPosition.Goal().Translation().X() = currPosition.Position().Translation().X() + 0.001;
        newPosition.Goal().Translation().Y() = currPosition.Position().Translation().Y();
        newPosition.Goal().Translation().Z() = currPosition.Position().Translation().Z();
        vctQuatRot3 quatNew;
        quatNew.X() = quatCurr.X();
        quatNew.Y() = quatCurr.Y();
        quatNew.Z() = quatCurr.Z();
        quatNew.W() = quatCurr.W();
        vctMatRot3 rotation(quatNew, VCT_NORMALIZE);
        newPosition.Goal().Rotation().FromNormalized(rotation);
        if (counter == 50) {
            totalTranslation += 0.001;
            Generator.SetPositionCartesian(newPosition);
            counter = 0;
        }
    }
}


void mtsIntuitiveResearchKitBlock::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveResearchKitBlock::Startup(void)
{
}

void mtsIntuitiveResearchKitBlock::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
    Generator.GetPositionCartesian(currPosition);
    Generator.GetRobotState(RobotState);
    if (RobotState == true) {
        RunPositionCartesian();
    }
}

void mtsIntuitiveResearchKitBlock::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << ": Cleanup" << std::endl;
}
