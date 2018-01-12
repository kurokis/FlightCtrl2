#include "nav_comms.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/crc16.h>
#include <string.h>

#include "adc.h"
#include "attitude.h"
#include "control.h"
#include "main.h"
#include "pressure_altitude.h"
#include "state.h"
#include "sbus.h"
#include "ut_serial_protocol.h"
#include "timing.h"
#include "union_types.h"
#include "vertical_speed.h"

#include "motors.h"

// =============================================================================
// Private data:

#define NAV_COMMS_VERSION (1)
#define NAV_FRESHNESS_LIMIT (500)  // milliseconds
#define NAV_MESSAGE_START_BYTE (0xAA)

static volatile struct FromNav {
    uint16_t version;
    uint8_t nav_mode;
    uint8_t status;
    float position[3];
    float velocity[3];
    float heading_correction_quat_0;
    float heading_correction_quat_z;
    float target_position[3];
    float transit_speed;
    float target_heading;
    float heading_rate;
    //uint16_t crc;
} __attribute__((packed)) from_nav_;

static enum NavModeBits {
  NAV_BIT_MODE_0     = 1<<0,
  NAV_BIT_MODE_1     = 1<<1,
  NAV_BIT_HOLD_RESET = 1<<2,
  NAV_BIT_RESERVED_0 = 1<<3,
  NAV_BIT_ROUTE_0    = 1<<4,
  NAV_BIT_ROUTE_1    = 1<<5,
  NAV_BIT_SWITCH_0   = 1<<6,
  NAV_BIT_SWITCH_1   = 1<<7,
} nav_mode_request_;

static uint16_t last_reception_timestamp_ = 0;
static enum NavErrorBits nav_error_bits_ = NAV_ERROR_BIT_STALE;

// =============================================================================
// Accessors:

// -----------------------------------------------------------------------------
enum NavMode NavMode(void)
{
  return (enum NavMode)(from_nav_.nav_mode & 0x03);
}

// -----------------------------------------------------------------------------
uint8_t NavPositionReset(void)
{
  return from_nav_.status
    & NAV_STATUS_BIT_POSITION_RESET_REQUEST;
}

// -----------------------------------------------------------------------------
uint8_t NavStatus(void)
{
  return from_nav_.status;
}

// -----------------------------------------------------------------------------
uint8_t NavStatusOK(void)
{
  uint8_t ok = NAV_STATUS_BIT_HEADING_DATA_OK | NAV_STATUS_BIT_POSITION_DATA_OK
    | NAV_STATUS_BIT_VELOCITY_DATA_OK;
  return ((from_nav_.status & ok) == ok) & !NavStale();
}

// -----------------------------------------------------------------------------
float HeadingCorrection0(void)
{
  // Limit values of heading correction to fulfill linear approximation
  float hc0 = from_nav_.heading_correction_quat_0;
  float hcz = from_nav_.heading_correction_quat_z;
  if(fabs(hcz) > 0.05){
    hc0 = 0.9987492178; // sqrt(1-0.05^2)
  }
  return hc0;
}

// -----------------------------------------------------------------------------
float HeadingCorrectionZ(void)
{
  // Limit values of heading correction to fulfill linear approximation
  float hcz = from_nav_.heading_correction_quat_z;
  if(fabs(hcz) > 0.05){
    if(hcz > 0){
      hcz = 0.05;
    }else{
      hcz = -0.05;
    }
  }
  return hcz;
}

// -----------------------------------------------------------------------------
const volatile float * PositionVector(void)
{
  return from_nav_.position;
}

// -----------------------------------------------------------------------------
const volatile float * VelocityVector(void)
{
  return from_nav_.velocity;
}

// -----------------------------------------------------------------------------
const volatile float * TargetPositionVector(void)
{
  return from_nav_.target_position;
}

// -----------------------------------------------------------------------------
float TransitSpeed(void)
{
  return from_nav_.transit_speed;
}

// -----------------------------------------------------------------------------
float TargetHeading(void)
{
  return from_nav_.target_heading;
}

// -----------------------------------------------------------------------------
float HeadingRate(void)
{
  return from_nav_.heading_rate;
}

// -----------------------------------------------------------------------------
uint8_t NavStale(void)
{
  return nav_error_bits_ & NAV_ERROR_BIT_STALE;
}


// =============================================================================
// Public functions:

void SendDataToNav(void)
{
  // Only check freshness if the data is not yet stale because the timestamp
  // might rollover, giving a false freshness.
  if ((~nav_error_bits_ & NAV_ERROR_BIT_STALE) &&
    (MillisSinceTimestamp(last_reception_timestamp_) > NAV_FRESHNESS_LIMIT))
  {
    nav_error_bits_ |= NAV_ERROR_BIT_STALE;
  }

  // Specify the payload structure.
  struct ToNav {
    uint16_t timestamp;
    uint8_t nav_mode_request;
    uint8_t state;
    float accelerometer[3];
    float gyro[3];
    float quaternion[4];
    float pressure_altitude;
#ifdef DEBUG
    float g_b_cmd[2];
    float g_b_cmd_ad[2];
#endif
  } __attribute__((packed)) to_nav_;

  struct ToNav * to_nav_ptr;
  to_nav_ptr = &to_nav_;


  to_nav_ptr->timestamp = GetTimestamp();
  to_nav_ptr->nav_mode_request = nav_mode_request_ | NavModeRequest()
    | (SBusSwitch(0) << 4) | (SBusSwitch(1) << 6);
  to_nav_ptr->state = State();
  to_nav_ptr->accelerometer[0] = Acceleration(X_BODY_AXIS);
  to_nav_ptr->accelerometer[1] = Acceleration(Y_BODY_AXIS);
  to_nav_ptr->accelerometer[2] = Acceleration(Z_BODY_AXIS);
  to_nav_ptr->gyro[0] = AngularRate(X_BODY_AXIS);
  to_nav_ptr->gyro[1] = AngularRate(Y_BODY_AXIS);
  to_nav_ptr->gyro[2] = AngularRate(Z_BODY_AXIS);
  to_nav_ptr->quaternion[0] = Quat()[0];
  to_nav_ptr->quaternion[1] = Quat()[1];
  to_nav_ptr->quaternion[2] = Quat()[2];
  to_nav_ptr->quaternion[3] = Quat()[3];
  to_nav_ptr->pressure_altitude = DeltaPressureAltitude();
#ifdef DEBUG
  to_nav_ptr->g_b_cmd[0] = NavGBCommand()[0];
  to_nav_ptr->g_b_cmd[1] = NavGBCommand()[1];
  to_nav_ptr->g_b_cmd_ad[0] = AdaptiveGBCommand()[0];
  to_nav_ptr->g_b_cmd_ad[1] = AdaptiveGBCommand()[1];
#endif

  UTSerialTx(UT_SERIAL_ID_NAV, (uint8_t *) to_nav_ptr, sizeof(struct ToNav));
}

// -----------------------------------------------------------------------------
void ProcessDataFromNav(const uint8_t * data_buffer)
{
  struct FromNav * from_nav_data_buffer;
  from_nav_data_buffer = (struct FromNav *)data_buffer;


  if(from_nav_data_buffer->version == NAV_COMMS_VERSION){
    // Copy data from data_buffer
    volatile struct FromNav * from_nav_ptr;
    from_nav_ptr = &from_nav_;
    memcpy((uint8_t *)from_nav_ptr,data_buffer, sizeof(struct FromNav));

    // Clear the stale data bit.
    nav_error_bits_ &= ~NAV_ERROR_BIT_STALE;
    last_reception_timestamp_ = GetTimestamp();
  }

  // Clear the nav hold reset request if it has been honored.
  // TODO: maybe this should get handled elsewhere?
  if (from_nav_.nav_mode & NAV_BIT_HOLD_RESET)
    nav_mode_request_ &= ~NAV_BIT_HOLD_RESET;
}

// -----------------------------------------------------------------------------
void RequestNavRoute(uint8_t nav_route)
{
  if (nav_route > 0x03) nav_route = 0x03;
  nav_mode_request_ |= nav_route << 4;
}

// -----------------------------------------------------------------------------
void ResetPositionHold(void)
{
  nav_mode_request_ |= NAV_BIT_HOLD_RESET;
}
