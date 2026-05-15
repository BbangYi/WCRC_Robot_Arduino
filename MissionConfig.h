#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include "MissionRouteTuner/WcrcMissionSharedConfig.h"

/*
 * Motor-specific mission config wrapper.
 *
 * Shared field calibration lives in MissionRouteTuner/WcrcMissionSharedConfig.h.
 * Keep only values that are unique to the fully autonomous Motor mission here.
 */
class MissionConfig : public WcrcMissionSharedConfig
{
public:
  struct PoseTimingConfig
  {
    // Manipulator profile times. Smaller values are faster but increase shake.
    uint16_t startupInitialMs = 1200;
    uint16_t missionInstructionMs = 800;
    uint16_t storageMs = 700;
    uint16_t preGripMs = 750;
    uint16_t gripMs = 850;
    uint16_t upperGripStageMs = 280;
    uint16_t upperLiftStageMs = 320;
    uint16_t storageWithBlockMs = 800;
    uint16_t missionZoneTurnMs = 650;
    uint16_t missionZonePlaceMs = 800;
    uint16_t finishStorageMs = 650;
    uint16_t finishInitialMs = 800;
  };

  struct WaitConfig
  {
    uint16_t buttonDebounceMs = 80;
    uint16_t driveSettleMs = 50;
    uint16_t blockFeedbackMs = 80;
    uint16_t scanSampleMs = 20;
    uint16_t cameraLampMs = 150;
    uint16_t gripperActionMs = 250;
    uint16_t poseSettleMs = 100;
  };

  PoseTimingConfig poseTiming;
  WaitConfig wait;

  static const MissionConfig &get()
  {
    static const MissionConfig instance;
    return instance;
  }

private:
  MissionConfig() {}
  MissionConfig(const MissionConfig &);
  MissionConfig &operator=(const MissionConfig &);
};

#endif
