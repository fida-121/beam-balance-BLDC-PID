#include "pid.h"
#include <math.h>

/* ===== INTERNAL STATE ===== */
static float pid_integral   = 0.0f;
static float d_filt         = 0.0f;    /* Low-pass filtered derivative state */
static float stable_time    = 0.0f;    /* Seconds within inner deadband       */

/* ===== RESET ALL STATE ===== */
void pid_reset(void)
{
    pid_integral = 0.0f;
    d_filt       = 0.0f;
    stable_time  = 0.0f;
}

/* ===== PID COMPUTE =====
   Returns a signed motor adjustment value.
   Positive  -> beam nose up  -> increase motor 2 / decrease motor 1.
   Negative  -> beam nose down -> vice versa.

   Features:
     - Inner/outer deadband with smooth linear ramp
     - Integral only accumulates within ±3° of target
     - Integral slowly decays when system is stable (helps remove offset)
     - Derivative computed on measurement (not error) to avoid kick on
       setpoint changes, then low-pass filtered at 0.7 / 0.3
*/
float pid_compute(float setpoint, float measured, float dt)
{
    float error     = setpoint - measured;
    float abs_error = fabsf(error);

    /* ---- DEADBAND SCALE ---- */
    float deadband_scale;
    if (abs_error < DEADBAND_INNER) {
        deadband_scale = 0.0f;
    } else if (abs_error < DEADBAND_OUTER) {
        deadband_scale = (abs_error - DEADBAND_INNER) /
                         (DEADBAND_OUTER - DEADBAND_INNER);
    } else {
        deadband_scale = 1.0f;
    }

    /* ---- INTEGRAL DECAY WHEN STABLE ---- */
    if (abs_error < DEADBAND_INNER) {
        stable_time += dt;
        if (stable_time > 1.0f) {
            pid_integral *= 0.80f;   /* Slow windup bleed */
        }
    } else {
        stable_time = 0.0f;
    }

    /* ---- PROPORTIONAL ---- */
    float p_term = KP * error;

    /* ---- INTEGRAL (only near balance point) ---- */
    if (fabsf(error) < 3.0f) {
        pid_integral += error * dt;
    }
    float i_term = KI * pid_integral;

    /* ---- DERIVATIVE ON MEASUREMENT + LPF ---- */
    float raw_d = (measured - d_filt) / dt;
    d_filt      = 0.7f * d_filt + 0.3f * raw_d;   /* LPF coefficient */
    float d_term = -KD * d_filt;

    /* Update derivative state to current measurement */
    d_filt = measured;

    /* ---- COMBINE AND APPLY DEADBAND ---- */
    float output = deadband_scale * (p_term + i_term + d_term);

    return output;
}
