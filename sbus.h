#ifndef SBUS_H_
#define SBUS_H_


// =============================================================================
// Definitions:

#define SBUS_MESSAGE_LENGTH (25 - 1)  // Start byte is not recorded
#define SBUS_RX_BUFFER_LENGTH (SBUS_MESSAGE_LENGTH + 2)  // Includes timestamp
#define SBUS_START_BYTE (0x0F)
#define SBUS_END_BYTE (0x00)
#define SBUS_MAX (672)


#ifndef __ASSEMBLER__


#include <inttypes.h>


enum SBusErrorBits {
  SBUS_ERROR_BIT_STALE = 1<<0,
};

enum SBusSwitchState {
  SBUS_SWITCH_DOWN = 0,
  SBUS_SWITCH_CENTER = 1,
  SBUS_SWITCH_UP = 2,
};


// =============================================================================
// Accessors:

uint8_t SBusErrorBits(void);

// -----------------------------------------------------------------------------
int16_t SBusPitch(void);

// -----------------------------------------------------------------------------
int16_t SBusRoll(void);

// -----------------------------------------------------------------------------
int16_t SBusYaw(void);

// -----------------------------------------------------------------------------
int16_t SBusThrust(void);

// -----------------------------------------------------------------------------
uint8_t SBusOnOff(void);

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusAltitudeControl(void);

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusGoHome(void);

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusNavControl(void);

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusTakeoff(void);

// -----------------------------------------------------------------------------
enum SBusSwitchState SBusSwitch(uint8_t i);

// -----------------------------------------------------------------------------
int16_t SBusTrim(uint8_t i);

// -----------------------------------------------------------------------------
uint8_t SBusStale(void);


// =============================================================================
// Public functions:

void SBusInit(void);

// -----------------------------------------------------------------------------
void SBusSetChannels(uint8_t pitch, uint8_t roll, uint8_t yaw, uint8_t thrust,
  uint8_t on_off, uint8_t altitude_control, uint8_t nav_control,
  uint8_t takeoff, uint8_t go_home, uint8_t switch0, uint8_t switch1,
  uint8_t switch2, uint8_t switch3, uint8_t switch4, uint8_t trim0,
  uint8_t trim1, uint8_t trim2, uint8_t trim3);

// -----------------------------------------------------------------------------
uint8_t SBusPitchStickCentered(void);

// -----------------------------------------------------------------------------
uint8_t SBusRollStickCentered(void);

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickCentered(void);

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickDown(void);

// -----------------------------------------------------------------------------
uint8_t SBusThrustStickUp(void);

// -----------------------------------------------------------------------------
uint8_t SBusYawStickCentered(void);

// -----------------------------------------------------------------------------
uint8_t SBusYawStickLeft(void);

// -----------------------------------------------------------------------------
uint8_t SBusYawStickRight(void);

// -----------------------------------------------------------------------------
void UpdateSBus(void);


#endif  // __ASSEMBLER__

#endif  // SBUS_H_
