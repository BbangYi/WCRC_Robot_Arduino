#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include "WcrcMissionSharedConfig.h"

/*
 * MissionRouteTuner-specific config wrapper.
 *
 * Shared field calibration lives in WcrcMissionSharedConfig.h. Keep only
 * console/test timing values that differ from the autonomous Motor mission here.
 */
class MissionConfig : public WcrcMissionSharedConfig
{
public:
  struct WaitConfig
  {
    uint16_t buttonDebounceMs = 120;
    uint16_t driveSettleMs = 80;
    uint16_t blockFeedbackMs = 120;
    uint16_t scanSampleMs = 30;
    uint16_t cameraLampMs = 250;
    uint16_t gripperActionMs = 350;
    uint16_t pose800Ms = 850;
    uint16_t pose1000Ms = 1050;
    uint16_t pose1300Ms = 1100;
    uint16_t pose1500Ms = 1200;
  };

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
