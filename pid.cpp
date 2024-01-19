/**
 * @file pid.cpp
 * @author Marko
 * @brief Generic PID controller class source code.
 * @version 1.0
 * @date 2024-01-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "pid.h"

/**
 * @brief Constructor for PID class.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param min Minimum output value.
 * @param max Maximum output value.
 */
PID::PID(float kp, float ki, float kd, float min, float max) : _kp(kp),
                                                               _ki(ki),
                                                               _kd(kd),
                                                               _mino(min),
                                                               _maxo(max)
{
}

/**
 * @brief Destructor for PID class.
 */
PID::~PID()
{
}

/**
 * @brief Constrain a value within the specified limits.
 * @param value The value to constrain.
 * @return The constrained value.
 */
float PID::_constrain(float value)
{
    return (_mino < value && value < _maxo) ? value : ((value < _mino) ? _mino : _maxo);
}

/**
 * @brief Perform a PID control step.
 * @param state Current state of the system.
 * @param time Current timestamp.
 * @return Calculated PID output.
 */
float PID::tick(float state, unsigned long time)
{
    unsigned long delta = time - _lastTick;
    _lastTick = time;
    float dt = (float)(delta / TIME_DIV);
    float error = setpoint - state;
    float p = error * _kp;
    _integral += error * dt;
    float i = _integral * _ki;
    float d = ((error - _prevError) / dt) * _kd;
    _prevError = error;

    if (_mino != 0.0f || _maxo != 0.0f)
    {
        if (p + d > _mino && p + d < _maxo)
        {
            return _constrain(p + i + d);
        }
        else
        {
            _integral -= error * dt;
            return _constrain(p + d);
        }
    }
    else
    {
        return p + i + d;
    }
}

/**
 * @brief Reset the PID controller (set previous error and integral to 0).
 */
void PID::reset(void)
{
    _prevError = 0;
    _integral = 0;
}

/**
 * @brief Overwrite the current PID integral value.
 * @param value The new integral value.
 */
void PID::overwrite(float value)
{
    _integral = value;
}

/**
 * @brief Reset the PID controller with new gains.
 * @param kp New proportional gain.
 * @param ki New integral gain.
 * @param kd New derivative gain.
 */
void PID::reset(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}
