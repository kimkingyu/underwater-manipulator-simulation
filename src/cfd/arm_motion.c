/*****************************************************************************
 * arm_motion.c
 * 3-DOF underwater arm: Fluent rigid dynamic mesh UDF (DEFINE_CG_MOTION).
 * Keep this file ASCII-only: Fluent's Windows build scripts may read it as GBK.
 *
 * Parameters come from arm_motion_config.h. The default motion matches the
 * nominal MATLAB trajectory; custom cases use the supplied configuration.
 *   A: smoothly deploy from q_stow to q_work.
 *   B: sweep joint 1 from q1_work and back; hold q2 and q3 at q_work.
 *   C: smoothly retract from q_work to q_stow.
 * Segment boundary velocities are zero (C1-continuous trajectory).
 * Edit cfd_case_parameters.m and run prepare_fluent_udf.m to stage a case.
 *
 * Initial dynamic-zone reference points, in Fluent CAD coordinates [m]:
 *   lk1: joint1_motion, [-0.745634, 1.155872, -1.142240]
 *   lk2: joint2_motion, [-0.728944, 1.206372, -1.172240]
 *   lk3w1..lk3w4: joint3_motion, [-0.730834, 1.008372, -1.228240]
 * These reference points must match the mesh's initial joint locations.
 *****************************************************************************/

#ifndef UDF_STANDALONE_TEST
#include "udf.h"
#endif
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "arm_motion_config.h"

/* Motion and R12/R23 constants are supplied by arm_motion_config.h. */


/* Vector cross product: c = a x b. */
void cross_product(real *a, real *b, real *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/* Flat 3x3 matrices avoid multidimensional arrays in the Fluent interpreter. */
void mat3_mul(real *A, real *B, real *C)
{
    int i, j, k;
    real sum;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0;
            for (k = 0; k < 3; k++) {
                sum += A[3 * i + k] * B[3 * k + j];
            }
            C[3 * i + j] = sum;
        }
    }
}

/* Matrix-vector product: y = A * x. */
void mat3_vec(real *A, real *x, real *y)
{
    int i, k;
    real sum;

    for (i = 0; i < 3; i++) {
        sum = 0.0;
        for (k = 0; k < 3; k++) {
            sum += A[3 * i + k] * x[k];
        }
        y[i] = sum;
    }
}

static int arm_motion_is_finite(real value)
{
    return (value == value) && (value <= 1.0e300) && (value >= -1.0e300);
}

int arm_motion_config_is_valid(void)
{
    real geometry[] = {
        ARM_FIXED_R12_X_M, ARM_FIXED_R12_Y_M, ARM_FIXED_R12_Z_M,
        ARM_FIXED_R23_X_M, ARM_FIXED_R23_Y_M, ARM_FIXED_R23_Z_M
    };
    int i;

    if (!arm_motion_is_finite(ARM_CFG_T_DEPLOY_S) || !arm_motion_is_finite(ARM_CFG_T_WORK_S) ||
        !arm_motion_is_finite(ARM_CFG_T_RETRACT_S) || ARM_CFG_T_DEPLOY_S <= 0.0 ||
        ARM_CFG_T_WORK_S <= 0.0 || ARM_CFG_T_RETRACT_S <= 0.0)
        return 0;
    if (!arm_motion_is_finite(ARM_CFG_TIME_STEP_S) || ARM_CFG_TIME_STEP_S <= 0.0)
        return 0;
    if (!arm_motion_is_finite(ARM_CFG_Q1_SWEEP_HALF_DEG) || ARM_CFG_Q1_SWEEP_HALF_DEG < 0.0)
        return 0;
    if (ARM_CFG_Q1_SWEEP_DIRECTION != 1.0 && ARM_CFG_Q1_SWEEP_DIRECTION != -1.0)
        return 0;
    if (!arm_motion_is_finite(1.0 * ARM_CFG_T_TOTAL_S) ||
        fabs((1.0 * ARM_CFG_T_TOTAL_S) - (ARM_CFG_T_DEPLOY_S + ARM_CFG_T_WORK_S + ARM_CFG_T_RETRACT_S)) > 1e-12)
        return 0;
    for (i = 0; i < (int)(sizeof(geometry) / sizeof(geometry[0])); ++i) {
        if (!arm_motion_is_finite(geometry[i]))
            return 0;
    }
    return 1;
}

/*  (rad) */
void get_joint_trajectory(real t,
                                real *q1,  real *q2,  real *q3,
                                real *qd1, real *qd2, real *qd3)
{
    real u, s, sd, tw;
    real q1_sweep;

    q1_sweep = ARM_CFG_Q1_SWEEP_HALF_RAD * ARM_CFG_Q1_SWEEP_DIRECTION;
    u = 0.0;
    s = 0.0;
    sd = 0.0;
    tw = 0.0;

    if (t < 0.0) {
        *q1 = ARM_CFG_Q1_STOW_RAD;
        *q2 = ARM_CFG_Q2_STOW_RAD;
        *q3 = ARM_CFG_Q3_STOW_RAD;
        *qd1 = 0.0; *qd2 = 0.0; *qd3 = 0.0;
    }
    else if (t <= ARM_CFG_T_DEPLOY_S) {
        /* A:  (0 ~ T_DEPLOY) */
        u = t / ARM_CFG_T_DEPLOY_S;
        s = 0.5 * (1.0 - cos(ARM_MOTION_PI * u));
        sd = 0.5 * (ARM_MOTION_PI / ARM_CFG_T_DEPLOY_S) * sin(ARM_MOTION_PI * u);

        *q1  = ARM_CFG_Q1_STOW_RAD + (ARM_CFG_Q1_WORK_RAD - ARM_CFG_Q1_STOW_RAD) * s;
        *q2  = ARM_CFG_Q2_STOW_RAD + (ARM_CFG_Q2_WORK_RAD - ARM_CFG_Q2_STOW_RAD) * s;
        *q3  = ARM_CFG_Q3_STOW_RAD + (ARM_CFG_Q3_WORK_RAD - ARM_CFG_Q3_STOW_RAD) * s;
        *qd1 = (ARM_CFG_Q1_WORK_RAD - ARM_CFG_Q1_STOW_RAD) * sd;
        *qd2 = (ARM_CFG_Q2_WORK_RAD - ARM_CFG_Q2_STOW_RAD) * sd;
        *qd3 = (ARM_CFG_Q3_WORK_RAD - ARM_CFG_Q3_STOW_RAD) * sd;
    }
    else if (t <= (ARM_CFG_T_DEPLOY_S + ARM_CFG_T_WORK_S)) {
        /* B:  (T_DEPLOY ~ T_DEPLOY + T_WORK) */
        tw = t - ARM_CFG_T_DEPLOY_S;
        *q1  = ARM_CFG_Q1_WORK_RAD + q1_sweep * (1.0 - cos(ARM_CFG_OMEGA_B_RAD_S * tw));
        *qd1 = q1_sweep * ARM_CFG_OMEGA_B_RAD_S * sin(ARM_CFG_OMEGA_B_RAD_S * tw);
        *q2  = ARM_CFG_Q2_WORK_RAD;
        *qd2 = 0.0;
        *q3  = ARM_CFG_Q3_WORK_RAD;
        *qd3 = 0.0;
    }
    else if (t <= ARM_CFG_T_TOTAL_S) {
        /* C:  (T_DEPLOY + T_WORK ~ T_TOTAL) */
        u = (t - ARM_CFG_T_DEPLOY_S - ARM_CFG_T_WORK_S) / ARM_CFG_T_RETRACT_S;
        s = 0.5 * (1.0 - cos(ARM_MOTION_PI * u));
        sd = 0.5 * (ARM_MOTION_PI / ARM_CFG_T_RETRACT_S) * sin(ARM_MOTION_PI * u);
        *q1  = ARM_CFG_Q1_WORK_RAD + (ARM_CFG_Q1_STOW_RAD - ARM_CFG_Q1_WORK_RAD) * s;
        *q2  = ARM_CFG_Q2_WORK_RAD + (ARM_CFG_Q2_STOW_RAD - ARM_CFG_Q2_WORK_RAD) * s;
        *q3  = ARM_CFG_Q3_WORK_RAD + (ARM_CFG_Q3_STOW_RAD - ARM_CFG_Q3_WORK_RAD) * s;
        *qd1 = (ARM_CFG_Q1_STOW_RAD - ARM_CFG_Q1_WORK_RAD) * sd;
        *qd2 = (ARM_CFG_Q2_STOW_RAD - ARM_CFG_Q2_WORK_RAD) * sd;
        *qd3 = (ARM_CFG_Q3_STOW_RAD - ARM_CFG_Q3_WORK_RAD) * sd;
    }
    else {
        /* T_TOTAL  */
        *q1 = ARM_CFG_Q1_STOW_RAD;
        *q2 = ARM_CFG_Q2_STOW_RAD;
        *q3 = ARM_CFG_Q3_STOW_RAD;
        *qd1 = 0.0; *qd2 = 0.0; *qd3 = 0.0;
    }
}

/* Absolute rigid-body linear and angular velocities in the global frame. */
void calc_link_motion(real t, int link_idx, real *vel, real *omega)
{
    real q1, q2, q3, qd1, qd2, qd3;
    real c1, s1, c2, s2;
    real Ry1[9];
    real w1[3], v1[3];
    real a2[3], w2[3], v2[3], r12[3], r12_0_local[3];
    real Rx_m90[9], Rz_q1[9], Ry_m90[9], Rz_q2[9];
    real R1[9], temp1[9], R2[9];
    real axis3_local[3], a3[3];
    real w3[3], v3[3], r23[3], r23_2_local[3], v_rel[3];

    if (!arm_motion_config_is_valid()) {
        vel[0] = 0.0; vel[1] = 0.0; vel[2] = 0.0;
        omega[0] = 0.0; omega[1] = 0.0; omega[2] = 0.0;
        return;
    }

    get_joint_trajectory(t, &q1, &q2, &q3, &qd1, &qd2, &qd3);

    c1 = cos(q1); s1 = sin(q1);
    c2 = cos(q2); s2 = sin(q2);

    /* Link 1: turntable rotation about the global Y axis. */
    w1[0] = 0.0; w1[1] = qd1; w1[2] = 0.0;
    v1[0] = 0.0; v1[1] = 0.0; v1[2] = 0.0;

    if (link_idx == 1) {
        vel[0] = v1[0]; vel[1] = v1[1]; vel[2] = v1[2];
        omega[0] = w1[0]; omega[1] = w1[1]; omega[2] = w1[2];
        return;
    }

    /* Link 2: upper arm. */
    Ry1[0] = c1;  Ry1[1] = 0.0; Ry1[2] = s1;
    Ry1[3] = 0.0; Ry1[4] = 1.0; Ry1[5] = 0.0;
    Ry1[6] = -s1; Ry1[7] = 0.0; Ry1[8] = c1;

    a2[0] = -c1; a2[1] = 0.0; a2[2] = s1;
    w2[0] = w1[0] + qd2 * a2[0];
    w2[1] = w1[1] + qd2 * a2[1];
    w2[2] = w1[2] + qd2 * a2[2];
    r12_0_local[0] = ARM_FIXED_R12_X_M;
    r12_0_local[1] = ARM_FIXED_R12_Y_M;
    r12_0_local[2] = ARM_FIXED_R12_Z_M;
    mat3_vec(Ry1, r12_0_local, r12);
    cross_product(w1, r12, v2);

    if (link_idx == 2) {
        vel[0] = v2[0]; vel[1] = v2[1]; vel[2] = v2[2];
        omega[0] = w2[0]; omega[1] = w2[1]; omega[2] = w2[2];
        return;
    }

    /* Link 3: forearm and gripper. */
    Rx_m90[0] = 1.0; Rx_m90[1] = 0.0; Rx_m90[2] = 0.0;
    Rx_m90[3] = 0.0; Rx_m90[4] = 0.0; Rx_m90[5] = 1.0;
    Rx_m90[6] = 0.0; Rx_m90[7] = -1.0; Rx_m90[8] = 0.0;

    Rz_q1[0] = c1; Rz_q1[1] = -s1; Rz_q1[2] = 0.0;
    Rz_q1[3] = s1; Rz_q1[4] = c1;  Rz_q1[5] = 0.0;
    Rz_q1[6] = 0.0; Rz_q1[7] = 0.0; Rz_q1[8] = 1.0;

    Ry_m90[0] = 0.0; Ry_m90[1] = 0.0; Ry_m90[2] = -1.0;
    Ry_m90[3] = 0.0; Ry_m90[4] = 1.0; Ry_m90[5] = 0.0;
    Ry_m90[6] = 1.0; Ry_m90[7] = 0.0; Ry_m90[8] = 0.0;

    Rz_q2[0] = c2; Rz_q2[1] = -s2; Rz_q2[2] = 0.0;
    Rz_q2[3] = s2; Rz_q2[4] = c2;  Rz_q2[5] = 0.0;
    Rz_q2[6] = 0.0; Rz_q2[7] = 0.0; Rz_q2[8] = 1.0;

    mat3_mul(Rx_m90, Rz_q1, R1);
    mat3_mul(Ry_m90, Rz_q2, temp1);
    mat3_mul(R1, temp1, R2);

    axis3_local[0] = 0.0; axis3_local[1] = 0.0; axis3_local[2] = -1.0;
    mat3_vec(R2, axis3_local, a3);
    w3[0] = w2[0] + qd3 * a3[0];
    w3[1] = w2[1] + qd3 * a3[1];
    w3[2] = w2[2] + qd3 * a3[2];

    r23_2_local[0] = ARM_FIXED_R23_X_M;
    r23_2_local[1] = ARM_FIXED_R23_Y_M;
    r23_2_local[2] = ARM_FIXED_R23_Z_M;
    mat3_vec(R2, r23_2_local, r23);
    cross_product(w2, r23, v_rel);
    v3[0] = v2[0] + v_rel[0];
    v3[1] = v2[1] + v_rel[1];
    v3[2] = v2[2] + v_rel[2];

    if (link_idx == 3) {
        vel[0] = v3[0]; vel[1] = v3[1]; vel[2] = v3[2];
        omega[0] = w3[0]; omega[1] = w3[1]; omega[2] = w3[2];
        return;
    }
}

/* =========================================================================
 * Fluent DEFINE_CG_MOTION entry points
 * ========================================================================= */

/* Joint 1: turntable, lk1. */
DEFINE_CG_MOTION(joint1_motion, dt, cg_vel, cg_omega, time, dtime)
{
    real vel[3], omega[3];
    NV_S(cg_vel, =, 0.0);
    NV_S(cg_omega, =, 0.0);
    
    calc_link_motion(time, 1, vel, omega);
    
    cg_vel[0] = vel[0];
    cg_vel[1] = vel[1];
    cg_vel[2] = vel[2];
    
    cg_omega[0] = omega[0];
    cg_omega[1] = omega[1];
    cg_omega[2] = omega[2];
}

/* Joint 2: upper arm, lk2. */
DEFINE_CG_MOTION(joint2_motion, dt, cg_vel, cg_omega, time, dtime)
{
    real vel[3], omega[3];
    NV_S(cg_vel, =, 0.0);
    NV_S(cg_omega, =, 0.0);
    
    calc_link_motion(time, 2, vel, omega);
    
    cg_vel[0] = vel[0];
    cg_vel[1] = vel[1];
    cg_vel[2] = vel[2];
    
    cg_omega[0] = omega[0];
    cg_omega[1] = omega[1];
    cg_omega[2] = omega[2];
}

/* Joint 3: forearm and gripper, lk3w1..lk3w4. */
DEFINE_CG_MOTION(joint3_motion, dt, cg_vel, cg_omega, time, dtime)
{
    real vel[3], omega[3];
    NV_S(cg_vel, =, 0.0);
    NV_S(cg_omega, =, 0.0);
    
    calc_link_motion(time, 3, vel, omega);
    
    cg_vel[0] = vel[0];
    cg_vel[1] = vel[1];
    cg_vel[2] = vel[2];
    
    cg_omega[0] = omega[0];
    cg_omega[1] = omega[1];
    cg_omega[2] = omega[2];
}
