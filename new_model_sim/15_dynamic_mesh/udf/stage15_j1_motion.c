/* Stage 15: new AUVM CAD, one rigid locked arm, J1 axis -Z.
 *
 * REQUIRED SETUP: the dynamic zone rotation/reference point must be explicitly
 * set to (-0.000004, -0.0366, -0.460702) m and read back before calling this UDF.
 * It is a kinematic origin, NOT the arm mass centre or body origin Ob.  We check
 * DT_CG but never silently rewrite solver state.  With this origin, v = 0.
 *
 * Every output depends only on absolute callback time, not invocation count or
 * accumulated angle.  No Data_Valid_P/flow iteration is needed by this law.
 * The meaning of Fluent's time/dtime remains UNVERIFIED until the real probe;
 * do not add a half/full-step offset or claim exact integrated mesh position.
 *
 * Pure C functions can be compiled with -DSTAGE15_STANDALONE without udf.h.
 * Such a build verifies arithmetic only, NOT Fluent binding or time semantics.
 * Constants are frozen to contract.json; test_motion.py checks their agreement.
 */

#include <float.h>
#include <math.h>
#include <stddef.h>

#define STAGE15_PI 3.141592653589793238462643383279502884
#define STAGE15_Q10 0.225
#define STAGE15_Q20 -0.25
#define STAGE15_Q30 -0.20
#define STAGE15_AMPLITUDE 0.025
#define STAGE15_PERIOD 2.0
#define STAGE15_START_TIME 0.0
#define STAGE15_ORIGIN_X -0.000004
#define STAGE15_ORIGIN_Y -0.0366
#define STAGE15_ORIGIN_Z -0.460702
#define STAGE15_CG_TOL_M 1.0e-10

static int stage15_is_finite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

/* Return 0 on success, -1 on invalid input/output.  Arrays have length three. */
int stage15_joint_state(double time, double q[3], double qd[3], double qdd[3])
{
    double frequency, phase;
    if (q == NULL || qd == NULL || qdd == NULL || !stage15_is_finite(time))
        return -1;
    frequency = 2.0 * STAGE15_PI / STAGE15_PERIOD;
    phase = frequency * (time - STAGE15_START_TIME);
    if (!stage15_is_finite(phase))
        return -1;
    q[0] = STAGE15_Q10 + STAGE15_AMPLITUDE * (1.0 - cos(phase));
    q[1] = STAGE15_Q20;
    q[2] = STAGE15_Q30;
    qd[0] = STAGE15_AMPLITUDE * frequency * sin(phase);
    qd[1] = qd[2] = 0.0;
    qdd[0] = STAGE15_AMPLITUDE * frequency * frequency * cos(phase);
    qdd[1] = qdd[2] = 0.0;
    return stage15_is_finite(q[0]) && stage15_is_finite(qd[0])
        && stage15_is_finite(qdd[0]) ? 0 : -1;
}

/* Pure translational/angular callback output for the required J1 origin. */
int stage15_motion_at_time(double time, double velocity[3], double omega[3])
{
    double q[3], qd[3], qdd[3];
    if (velocity == NULL || omega == NULL)
        return -1;
    velocity[0] = velocity[1] = velocity[2] = 0.0;
    omega[0] = omega[1] = omega[2] = 0.0;
    if (stage15_joint_state(time, q, qd, qdd) != 0)
        return -1;
    omega[2] = -qd[0];
    return 0;
}

/* Independent analytic C(t), v(t) for a reference initially in the qe mesh.
 * Available for offline cross-checks only; the callback below does not enable
 * off-axis-reference fallback without a separately frozen setup decision.
 */
int stage15_j1_reference_position_velocity(double time, const double reference0[3],
                                          double position[3], double velocity[3])
{
    double q[3], qd[3], qdd[3], delta, c, s, dx, dy;
    if (reference0 == NULL || position == NULL || velocity == NULL)
        return -1;
    if (!stage15_is_finite(reference0[0]) || !stage15_is_finite(reference0[1])
        || !stage15_is_finite(reference0[2]))
        return -1;
    if (stage15_joint_state(time, q, qd, qdd) != 0)
        return -1;
    delta = q[0] - STAGE15_Q10;
    c = cos(delta);
    s = sin(delta);
    dx = reference0[0] - STAGE15_ORIGIN_X;
    dy = reference0[1] - STAGE15_ORIGIN_Y;
    /* Right-hand rotation by delta about -Z. */
    position[0] = STAGE15_ORIGIN_X + c * dx + s * dy;
    position[1] = STAGE15_ORIGIN_Y - s * dx + c * dy;
    position[2] = reference0[2];
    velocity[0] = qd[0] * (position[1] - STAGE15_ORIGIN_Y);
    velocity[1] = -qd[0] * (position[0] - STAGE15_ORIGIN_X);
    velocity[2] = 0.0;
    return stage15_is_finite(position[0]) && stage15_is_finite(position[1])
        && stage15_is_finite(position[2]) && stage15_is_finite(velocity[0])
        && stage15_is_finite(velocity[1]) ? 0 : -1;
}

#ifndef STAGE15_STANDALONE
#include "udf.h"

DEFINE_CG_MOTION(stage15_j1_motion, dt, vel, omega, time, dtime)
{
#if RP_3D
    double velocity_d[3], omega_d[3];
    double cg[3];
    int i;
    NV_S(vel, =, 0.0);
    NV_S(omega, =, 0.0);
    if (dt == NULL) {
        Error("STAGE15: missing dynamic-thread reference point.\n");
        return;
    }
    for (i = 0; i < 3; ++i)
        cg[i] = (double)DT_CG(dt)[i];
    if (!stage15_is_finite(cg[0]) || !stage15_is_finite(cg[1])
        || !stage15_is_finite(cg[2])
        || fabs(cg[0] - STAGE15_ORIGIN_X) > STAGE15_CG_TOL_M
        || fabs(cg[1] - STAGE15_ORIGIN_Y) > STAGE15_CG_TOL_M
        || fabs(cg[2] - STAGE15_ORIGIN_Z) > STAGE15_CG_TOL_M) {
        Error("STAGE15: DT_CG must be explicitly set/read back at J1 origin; "
              "actual=(%.17g,%.17g,%.17g).\n", cg[0], cg[1], cg[2]);
        return;
    }
    if (!stage15_is_finite((double)dtime) || dtime < 0.0
        || stage15_motion_at_time((double)time, velocity_d, omega_d) != 0) {
        Error("STAGE15: non-finite motion time/state or negative dtime.\n");
        return;
    }
    for (i = 0; i < 3; ++i) {
        vel[i] = (real)velocity_d[i];
        omega[i] = (real)omega_d[i];
    }
    Message0("STAGE15_MOTION time=%.17g dtime=%.17g "
             "DT_CG=(%.17g,%.17g,%.17g) omega=(%.17g,%.17g,%.17g)\n",
             (double)time, (double)dtime, cg[0], cg[1], cg[2],
             omega_d[0], omega_d[1], omega_d[2]);
#else
    NV_S(vel, =, 0.0);
    NV_S(omega, =, 0.0);
    Error("STAGE15: this new-CAD motion requires a 3-D Fluent case.\n");
#endif
}
#endif
