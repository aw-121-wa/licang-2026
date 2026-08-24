#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

typedef struct
{
    float front_left;
    float front_right;
    float rear_left;
    float rear_right;
} MecanumWheelValues;

/* Body axes: forward +X, left +Y, counter-clockwise +Omega. */
void MecanumKinematics_Solve(float forward, float left,
                             float counter_clockwise,
                             MecanumWheelValues *wheels);
float MecanumKinematics_DesaturateWithScale(MecanumWheelValues *wheels,
                                             float maximum_magnitude);
void MecanumKinematics_Desaturate(MecanumWheelValues *wheels,
                                  float maximum_magnitude);

#endif
