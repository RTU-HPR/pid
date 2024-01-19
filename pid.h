/**
 * @file pid.h
 * @author Marko
 * @brief Generic PID controller class.
 * @version 1.0
 * @date 2024-01-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define TIME_DIV 1000000.0f

/**
 * @brief PID class for implementing a generic PID controller.
 */
class PID
{
private:
    float _kp;               /**< Proportional gain. */
    float _ki;               /**< Integral gain. */
    float _kd;               /**< Derivative gain. */
    float _prevError;        /**< Previous error for derivative term. */
    float _integral;         /**< Integral term accumulator. */
    float _maxo;             /**< Maximum output value. */
    float _mino;             /**< Minimum output value. */
    unsigned long _lastTick; /**< Timestamp of the last update. */

    /**
     * @brief Constrain a value within the specified limits.
     * @param value The value to constrain.
     * @return The constrained value.
     */
    float _constrain(float value);

public:
    float setpoint; /**< Setpoint value for the PID controller. */

    /**
     * @brief Constructor for PID class.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param min Minimum output value (default: 0.0f).
     * @param max Maximum output value (default: 0.0f).
     */
    PID(float kp, float ki, float kd, float min = 0.0f, float max = 0.0f);

    /**
     * @brief Destructor for PID class.
     */
    ~PID();

    /**
     * @brief Perform a PID control step.
     * @param state Current state of the system.
     * @param time Current timestamp.
     * @return Calculated PID output.
     */
    float tick(float state, unsigned long time);

    /**
     * @brief Overwrite the current PID integral value.
     * @param value The new integral value.
     */
    void overwrite(float value);

    /**
     * @brief Reset the PID controller (set previous error and integral to 0).
     */
    void reset(void);

    /**
     * @brief Reset the PID controller with new gains.
     * @param kp New proportional gain.
     * @param ki New integral gain.
     * @param kd New derivative gain.
     */
    void reset(float kp, float ki, float kd);
};

#endif
