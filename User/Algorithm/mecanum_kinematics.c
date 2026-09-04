#include "mecanum_kinematics.h"

static float Mecanum_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

void MecanumKinematics_Solve(float forward, float left,
                             float counter_clockwise,
                             MecanumWheelValues *wheels)
{
    if (wheels == 0) { return; }
    wheels->front_left  = forward - left - counter_clockwise;
    wheels->front_right = forward + left + counter_clockwise;
    wheels->rear_left   = forward + left - counter_clockwise;
    wheels->rear_right  = forward - left + counter_clockwise;
}

float MecanumKinematics_DesaturateWithScale(MecanumWheelValues *wheels,
                                             float maximum_magnitude)
{
    float maximum;
    float value;
    float scale = 1.0f;
    if ((wheels == 0) || (maximum_magnitude <= 0.0f)) { return 1.0f; }
    maximum = Mecanum_Absolute(wheels->front_left);
    value = Mecanum_Absolute(wheels->front_right); if (value > maximum) { maximum = value; }
    value = Mecanum_Absolute(wheels->rear_left); if (value > maximum) { maximum = value; }
    value = Mecanum_Absolute(wheels->rear_right); if (value > maximum) { maximum = value; }
    if (maximum > maximum_magnitude)
    {
        scale = maximum_magnitude / maximum;
        wheels->front_left *= scale;
        wheels->front_right *= scale;
        wheels->rear_left *= scale;
        wheels->rear_right *= scale;
    }
    return scale;
}

void MecanumKinematics_Desaturate(MecanumWheelValues *wheels,
                                  float maximum_magnitude)
{
    (void)MecanumKinematics_DesaturateWithScale(wheels, maximum_magnitude);
}
