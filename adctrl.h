#ifndef __ADCTRL_H__
#define __ADCTRL_H__

// =============================================================================
// Public functions:

void InitializeAdctrl(void);

// These functions need to be called every Ts seconds
float AdaptivePitchControl(float* pitch_states, float nominal_pitch_command);
float AdaptiveRollControl(float* roll_states, float nominal_roll_command);
//float AdaptiveThrustControl(float* thrust_states, float nominal_z_command);
//float AdaptiveYawControl(float* yaw_states, float nominal_yaw_command);

#endif //__ADCTRL_H__
