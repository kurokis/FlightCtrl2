// Adaptive control

#include "adctrl.h"
#include "lpf.h"
#include "matrix.h"
#include "vector.h"

struct L1{
  float Ts;
  float Am[3 * 3];
  float Bm[3 * 1];
  float Bum[3 * 2];
  float B[3 * 3];
  float Kg;
  // Adaptive laws
  float InvPhi[3 * 3];
  float InvPhi_eAmTs[3 * 3];
  float x_predictor[3];
  float h[3];
  float sigma[3];
  // Control laws
  struct lpf12_params Hsigma1;
  struct lpf56_params Hsigma2;
  struct lpf56_params Hsigma3;
  float delta_command;
  float adaptive_command;
} l1x,l1y,l1z,l1psi;

// =============================================================================
// Declaration of private functions:
void InitializeL1(struct L1* l1, const float Ts_, const float* Am_,
  const float* Bm_, const float* Bum_, const float Kg_,
  const float* Hsigma1_sos, const float Hsigma1_gain,
  const float* Hsigma2_sos, const float Hsigma2_gain,
  const float* Hsigma3_sos, const float Hsigma3_gain);
float AdaptiveControl(struct L1* l1, float* x_observer, float nominal_command);
void LimitAbsoluteValue(float * f, float limit_value);

// =============================================================================
// Public functions:

void InitializeAdctrl(void){
  // Pitch control parameters (128 Hz)
  // Each row of sos is in the order of b0, b1, b2, a1, a2,
  // where H(z) = (b0 + b1 z^(-1) + b2 z^(-2))/(1 + a1 z^(-1) + a2 z^(-2)).
  const float Ts = 1.0 / 128.0;
  const float Amx[3 * 3] =
    { -36.90295082, -594.1758937, -3674.012659, 1, 0, 0, 0, 1, 0 };
  const float Bmx[3 * 1] = { 3674.012659, 0, 0 };
  const float Bumx[3 * 2] = { 0, 0, 99.5587772078336, 0, 0, 6.183375491929674 };
  const float Kgx = 1.0;
  const float Hsigma1x_sos[5] = { 1, 1, 0, -0.862606, 0 };
  const float Hsigma1x_gain = 0.068697033871630;
  const float Hsigma2x_sos[3 * 5] =
    { 1, 1, 0, -0.903714, 0,
      1, -1.65133, 0.675635, -1.81325, 0.829109,
      1, -1.81325, 0.829109, -1.97893, 0.979145 };
  const float Hsigma2x_gain = 0.000426671819881097;
  const float Hsigma3x_sos[3 * 5] =
    { 1, 0.0962854, -0.903715, -1.90224, 0.902382,
      1, -1.71806, 0.749611, -1.81325, 0.829109,
      1, -1.81325, 0.829109, -1.97893, 0.979145 };
  const float Hsigma3x_gain = 0.000005031069885030881;

  InitializeL1(&l1x, Ts, Amx, Bmx, Bumx, Kgx,
    Hsigma1x_sos, Hsigma1x_gain,
    Hsigma2x_sos, Hsigma2x_gain,
    Hsigma3x_sos, Hsigma3x_gain);

  InitializeL1(&l1y, Ts, Amx, Bmx, Bumx, Kgx,
    Hsigma1x_sos, Hsigma1x_gain,
    Hsigma2x_sos, Hsigma2x_gain,
    Hsigma3x_sos, Hsigma3x_gain);

  //InitializeL1(&l1z, Ts, Amz, Bmz, Bumz, Kgz,
  //  Hsigma1z_sos, Hsigma1z_gain,
  //  Hsigma2z_sos, Hsigma2z_gain,
  //  Hsigma3z_sos, Hsigma3z_gain);

  //InitializeL1(&l1psi, Ts, Ampsi, Bmpsi, Bumpsi, Kgpsi,
  //  Hsigma1psi_sos, Hsigma1psi_gain,
  //  Hsigma2psi_sos, Hsigma2psi_gain,
  //  Hsigma3psi_sos, Hsigma3psi_gain);
}

float AdaptivePitchControl(float* pitch_states, float nominal_pitch_command)
{
  // pitch_states: qdot, q, theta
  float adaptive_pitch_command = AdaptiveControl(&l1x, pitch_states,
    nominal_pitch_command);
  return adaptive_pitch_command;
}

float AdaptiveRollControl(float* roll_states, float nominal_roll_command)
{
  // roll_states: pdot, p, phi
  float adaptive_roll_command = AdaptiveControl(&l1y, roll_states,
    nominal_roll_command);
  return adaptive_roll_command;
}

//float AdaptiveThrustControl(float* thrust_states, float nominal_z_command)
//{
//  // thrust_states: wdot, w, z
//  float adaptive_thrust_command = AdaptiveControl(&l1z, thrust_states,
//    nominal_z_command);
//  return adaptive_thrust_command;
//}

//float AdaptiveYawControl(float* yaw_states, float nominal_yaw_command)
//{
//  // yaw_states: rdot, r, psi
//  float adaptive_yaw_command = AdaptiveControl(&l1psi, yaw_states,
//    nominal_yaw_command);
//  return adaptive_yaw_command;
//}

// =============================================================================
// Private functions:

void InitializeL1(struct L1* l1, const float Ts_, const float* Am_,
  const float* Bm_, const float* Bum_, const float Kg_,
  const float* Hsigma1_sos, const float Hsigma1_gain,
  const float* Hsigma2_sos, const float Hsigma2_gain,
  const float* Hsigma3_sos, const float Hsigma3_gain)
{
  l1->Ts = Ts_;
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      l1->Am[i * 3 + j] = Am_[i * 3 + j];
    }
    l1->Bm[i] = Bm_[i];
    l1->Bum[i * 2] = Bum_[i * 2];
    l1->Bum[i * 2 + 1] = Bum_[i * 2 + 1];
    l1->B[i * 3 + 0] = Bm_[i];
    l1->B[i * 3 + 1] = Bum_[i * 2];
    l1->B[i * 3 + 2] = Bum_[i * 2 + 1];
  }
  l1->Kg = Kg_;
  l1->delta_command = 0.0;
  l1->adaptive_command = 0.0;

  // e^(Am*Ts) = I + (Am*Ts) + (Am*Ts)^2 / 2 + (Am*Ts)^3 / 6 + ...
  float eAmTs[3 * 3];
  float coef = 1.0;
  float temp[3 * 3], temp2[3 * 3], temp3[3 * 3];
  SetMatrixToIdentity(eAmTs, 3);
  SetMatrixToIdentity(temp3, 3);

  MatrixScale(Am_, Ts_, 3, 3, temp);
  for (int i = 1; i < 5; i++){
    coef /= (float)i;
    MatrixMultiply(temp3, temp, 3, 3, 3, temp2); // (Am*Ts)^n
    MatrixCopy(temp2, 3, 3, temp3);
    MatrixScale(temp3, coef, 3, 3, temp2); // (Am*Ts)^n / n!
    MatrixAddToSelf(eAmTs, temp2, 3, 3);
  }

  // InvPhi = ( inv(Am)*(e^(Am*Ts) - eye(dim_Am))*B )^(-1)
  float eAmTs_m_eye[3 * 3];
  float* InvPhi = l1->InvPhi;
  float* InvPhi_eAmTs = l1->InvPhi_eAmTs;
  MatrixInverse(Am_, 3, temp);
  MatrixCopy(eAmTs, 3, 3, eAmTs_m_eye);
  MatrixSubtractIdentityFromSelf(eAmTs_m_eye, 3);
  MatrixMultiply(eAmTs_m_eye, l1->B, 3, 3, 3, temp2);
  MatrixMultiply(temp, temp2, 3, 3, 3, temp3);
  MatrixInverse(temp3, 3, InvPhi);
  MatrixMultiply(InvPhi, eAmTs, 3, 3, 3, InvPhi_eAmTs);

  // Control filters
  l1->Hsigma1.bq.b0 = Hsigma1_sos[0];
  l1->Hsigma1.bq.b1 = Hsigma1_sos[1];
  l1->Hsigma1.bq.b2 = Hsigma1_sos[2];
  l1->Hsigma1.bq.a1 = Hsigma1_sos[3];
  l1->Hsigma1.bq.a2 = Hsigma1_sos[4];
  l1->Hsigma1.bq.s1 = 0.0;
  l1->Hsigma1.bq.s2 = 0.0;
  l1->Hsigma1.g = Hsigma1_gain;

  l1->Hsigma2.bq[0].b0 = Hsigma2_sos[0];
  l1->Hsigma2.bq[0].b1 = Hsigma2_sos[1];
  l1->Hsigma2.bq[0].b2 = Hsigma2_sos[2];
  l1->Hsigma2.bq[0].a1 = Hsigma2_sos[3];
  l1->Hsigma2.bq[0].a2 = Hsigma2_sos[4];
  l1->Hsigma2.bq[0].s1 = 0.0;
  l1->Hsigma2.bq[0].s2 = 0.0;
  l1->Hsigma2.bq[1].b0 = Hsigma2_sos[5];
  l1->Hsigma2.bq[1].b1 = Hsigma2_sos[6];
  l1->Hsigma2.bq[1].b2 = Hsigma2_sos[7];
  l1->Hsigma2.bq[1].a1 = Hsigma2_sos[8];
  l1->Hsigma2.bq[1].a2 = Hsigma2_sos[9];
  l1->Hsigma2.bq[1].s1 = 0.0;
  l1->Hsigma2.bq[1].s2 = 0.0;
  l1->Hsigma2.bq[2].b0 = Hsigma2_sos[10];
  l1->Hsigma2.bq[2].b1 = Hsigma2_sos[11];
  l1->Hsigma2.bq[2].b2 = Hsigma2_sos[12];
  l1->Hsigma2.bq[2].a1 = Hsigma2_sos[13];
  l1->Hsigma2.bq[2].a2 = Hsigma2_sos[14];
  l1->Hsigma2.bq[2].s1 = 0.0;
  l1->Hsigma2.bq[2].s2 = 0.0;
  l1->Hsigma2.g = Hsigma2_gain;

  l1->Hsigma3.bq[0].b0 = Hsigma3_sos[0];
  l1->Hsigma3.bq[0].b1 = Hsigma3_sos[1];
  l1->Hsigma3.bq[0].b2 = Hsigma3_sos[2];
  l1->Hsigma3.bq[0].a1 = Hsigma3_sos[3];
  l1->Hsigma3.bq[0].a2 = Hsigma3_sos[4];
  l1->Hsigma3.bq[0].s1 = 0.0;
  l1->Hsigma3.bq[0].s2 = 0.0;
  l1->Hsigma3.bq[1].b0 = Hsigma3_sos[5];
  l1->Hsigma3.bq[1].b1 = Hsigma3_sos[6];
  l1->Hsigma3.bq[1].b2 = Hsigma3_sos[7];
  l1->Hsigma3.bq[1].a1 = Hsigma3_sos[8];
  l1->Hsigma3.bq[1].a2 = Hsigma3_sos[9];
  l1->Hsigma3.bq[1].s1 = 0.0;
  l1->Hsigma3.bq[1].s2 = 0.0;
  l1->Hsigma3.bq[2].b0 = Hsigma3_sos[10];
  l1->Hsigma3.bq[2].b1 = Hsigma3_sos[11];
  l1->Hsigma3.bq[2].b2 = Hsigma3_sos[12];
  l1->Hsigma3.bq[2].a1 = Hsigma3_sos[13];
  l1->Hsigma3.bq[2].a2 = Hsigma3_sos[14];
  l1->Hsigma3.bq[2].s1 = 0.0;
  l1->Hsigma3.bq[2].s2 = 0.0;
  l1->Hsigma3.g = Hsigma3_gain;
}

float AdaptiveControl(struct L1* l1, float* x_observer, float nominal_command)
{
  //------ Adaptive law ------
  float x_tilde[3];
  Vector3Subtract(l1->x_predictor, x_observer, x_tilde);
  Vector3SubtractFromSelf(l1->h, x_tilde); // update h

  // sigma = InvPhi*h - InvPhi*eAmTs*x_tilde
  float* sigma = l1->sigma;
  float temp[3];
  MatrixMultiply(l1->InvPhi, l1->h, 3, 3, 1, sigma);
  MatrixMultiply(l1->InvPhi_eAmTs, x_tilde, 3, 3, 1, temp);
  Vector3SubtractFromSelf(sigma, temp);

  //------ Control law ------
  float* delta_command = &l1->delta_command;
  temp[0] = lpf12(&l1->Hsigma1, sigma[0]);
  temp[1] = lpf56(&l1->Hsigma2, sigma[1]);
  temp[2] = lpf56(&l1->Hsigma3, sigma[2]);
  *delta_command = -1.0*(temp[0] + temp[1] + temp[2]);

  // Limit adaptive commands
  float delta_command_limit = 0.2;
  LimitAbsoluteValue(delta_command, delta_command_limit);

  l1->adaptive_command = *delta_command + nominal_command;

  //------ State predictor ------
  float temp2[3];
  float temp3 = (l1->Kg)*nominal_command + *delta_command + sigma[0];
  MatrixMultiply(l1->Bm, &temp3, 3, 1, 1, temp);
  MatrixMultiply(l1->Bum, &sigma[1], 3, 2, 1, temp2);

  float delta_x_predictor[3], x_predictor_nv[3];
  float* x_predictor_cv = l1->x_predictor; // current value
  MatrixMultiply(l1->Am, x_predictor_cv, 3, 3, 1, delta_x_predictor);
  Vector3AddToSelf(delta_x_predictor, temp);
  Vector3AddToSelf(delta_x_predictor, temp2);

  Vector3ScaleSelf(delta_x_predictor, l1->Ts);
  Vector3Add(x_predictor_cv, delta_x_predictor, x_predictor_nv);
  Vector3Copy(x_predictor_nv, l1->x_predictor); // update x_predictor

  return l1->adaptive_command;
}
void LimitAbsoluteValue(float * f, float limit_value)
{
  //if(limit_value < 0) return;

  if(*f > limit_value){
    *f = limit_value;
  }else if(*f < -limit_value){
    *f = -limit_value;
  }
}
