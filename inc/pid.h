#ifndef PID_H
#define PID_H

/* ===== TUNING PARAMETERS ===== */
#define KP              0.38f
#define KI              0.028f
#define KD              0.015f

#define TARGET_PITCH    0.0f    /* Desired balance angle (degrees) */

/* ===== DEADBAND ===== */
#define DEADBAND_INNER  0.8f    /* Error below this: zero output    */
#define DEADBAND_OUTER  1.4f    /* Error above this: full output     */
                                /* Between inner/outer: linear ramp  */

/* ===== PID LOOP FREQUENCY ===== */
#define PID_FREQ_HZ     200.0f
#define PID_PERIOD_S    (1.0f / PID_FREQ_HZ)

/* ===== FUNCTION ===== */
float pid_compute(float setpoint, float measured, float dt);
void  pid_reset(void);

#endif /* PID_H */
