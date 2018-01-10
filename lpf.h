#ifndef __LPF_H__
#define __LPF_H__

struct biquad_params{
	// H(z) = (b0 + b1 z^(-1) + b2 z^(-2))/(1 + a1 z^(-1) + a2 z^(-2))
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
	float s1;
	float s2;
};

struct lpf12_params{
	struct biquad_params bq; //second order section
	float g; // gain
};

struct lpf34_params{
	struct biquad_params bq[2]; //second order section
	float g; // gain
};

struct lpf56_params{
	struct biquad_params bq[3]; //second order section
	float g; // gain
};

float biquad(struct biquad_params* p, float x);
float lpf12(struct lpf12_params* p, float x); // discrete 1st or 2nd order filter
float lpf34(struct lpf34_params* p, float x); // discrete 3rd or 4th order filter
float lpf56(struct lpf56_params* p, float x); // discrete 5th or 6th order filter


#endif //__LPF_H__
