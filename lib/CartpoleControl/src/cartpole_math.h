#ifndef CARTPOLE_MATH_H
#define CARTPOLE_MATH_H

float CartpoleMath_Clamp(float value, float minimum, float maximum);
float CartpoleMath_WrapPi(float angle_rad);
float CartpoleMath_ThetaErrorFromUpright(float angle_rad);

#endif /* CARTPOLE_MATH_H */
