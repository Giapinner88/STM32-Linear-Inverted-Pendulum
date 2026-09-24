#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

float LqrController_Compute(float cart_position_ticks,
                            float cart_velocity_ticks_s,
                            float angle_rad,
                            float angle_rate_rad_s);

#endif /* LQR_CONTROLLER_H */
