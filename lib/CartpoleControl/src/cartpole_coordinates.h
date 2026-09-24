#ifndef CARTPOLE_COORDINATES_H
#define CARTPOLE_COORDINATES_H

/* The supplied Wheeltec controller defines positive cart motion toward the
 * right, while the raw TIM4 count increases toward the left. */
float CartpoleCoordinates_ToReferencePosition(float encoder_ticks);
float CartpoleCoordinates_ToReferenceVelocity(float encoder_ticks_per_second);

#endif /* CARTPOLE_COORDINATES_H */
