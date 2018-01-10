// Lowpass filter
#include "lpf.h"

float biquad(struct biquad_params * p, float x)
{
  // H(z) = (b0 + b1 z^(-1) + b2 z^(-2))/(1 + a1 z^(-1) + a2 z^(-2))
  // Transposed direct form II
  float y;
  y = p->b0 * x + p->s1;
  p->s1 = p->b1 * x + p->s2 - p->a1 * y;
  p->s2 = p->b2 * x - p->a2 * y;
  return y;
}

float lpf12(struct lpf12_params * p, float x)
{
  float y;
  y = biquad(&(p->bq), x);
  y *= p->g;
  return y;
}

float lpf34(struct lpf34_params * p, float x)
{
  float y;
  y = biquad(&(p->bq[0]), x);
  y = biquad(&(p->bq[1]), y);
  y *= p->g;
  return y;
}

float lpf56(struct lpf56_params * p, float x)
{
  float y;
  y = biquad(&(p->bq[0]), x);
  y = biquad(&(p->bq[1]), y);
  y = biquad(&(p->bq[2]), y);
  y *= p->g;
  return y;
}
