#include "state.h"

#include "adc.h"
#include "buzzer.h"
#include "pressure_altitude.h"
#include "sbus.h"
#include "timing.h"


// =============================================================================
// Private data:

static enum StateBits state_ = STATE_BIT_MOTORS_INHIBITED;


// =============================================================================
// Public functions:

void UpdateState(void)
{
  static uint8_t sbus_on_off_latch = 0;
  static uint16_t stick_timer;

  if (state_ & STATE_BIT_MOTORS_INHIBITED)
  {
    if (SBusThrustStickUp() && SBusYawStickLeft())
    {
      if (TimestampInPast(stick_timer))
      {
        ZeroGyros();
        ResetPressureSensorRange();
        state_ |= STATE_BIT_INITIALIZED;
        BeepDuration(500);
        stick_timer = GetTimestampMillisFromNow(2000);
      }
    }
    else if (SBusThrustStickUp() && SBusYawStickRight())
    {
      if (TimestampInPast(stick_timer))
      {
        ZeroAccelerometers();
        BeepDuration(500);
        stick_timer = GetTimestampMillisFromNow(2000);
      }
    }
    else if (SBusOnOff() && !sbus_on_off_latch && SBusThrustStickDown())
    {
      if (TimestampInPast(stick_timer))
      {
        state_ &= ~STATE_BIT_MOTORS_INHIBITED;
        BeepPattern(0x0000AAAA);
        sbus_on_off_latch = 1;
      }
    }
    else
    {
      stick_timer = GetTimestampMillisFromNow(1000);
    }
  }
  else
  {
    if (SBusOnOff() && !sbus_on_off_latch && SBusThrustStickDown())
    {
      if (TimestampInPast(stick_timer))
      {
        BeepPattern(0x0000AAAA);
        state_ |= STATE_BIT_MOTORS_INHIBITED;
        sbus_on_off_latch = 1;
      }
    }
    else
    {
      stick_timer = GetTimestampMillisFromNow(250);
    }
  }

  if (sbus_on_off_latch && !SBusOnOff()) sbus_on_off_latch = 0;
}
