/*****************************************************************************
 * arm_motion.c
 * ===========================================================================
 * 3-DOF  ANSYS Fluent  UDF (DEFINE_CG_MOTION)
 *
 * :
 *    MATLAB  (src/dynamics/calc_joint_loads.m)  100% :
 *   -  A  (0 ~ 3 s)   :  [0,0,0] -> [-30, -50, -55] deg
 *   -  B  (3 ~ 13 s)  :  q1 = -30*cos(w*t)  ( 60), /
 *   -  C  (13 ~ 16 s) : 
 *   -  (C1 ), 
 *
 * Fluent  (Dynamic Mesh Zones) :
 *   1. lk1 (, link_002):
 *      - Motion UDF: joint1_motion
 *      - Center of Gravity Location: [0.000000, 0.408011, -0.386600] m
 *   2. lk2 (, link_003):
 *      - Motion UDF: joint2_motion
 *      - Center of Gravity Location: [0.016690, 0.458511, -0.416600] m
 *   3. lk3w1 ~ lk3w4 (, link_004):
 *      - Motion UDF: joint3_motion
 *      - Center of Gravity Location: [0.014800, 0.260511, -0.472600] m
 *   ( CAD ,  CAD )
 *****************************************************************************/

#ifndef UDF_STANDALONE_TEST
#include "udf.h"
#endif
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*  ( MATLAB) */
#define T_DEPLOY   3.0
#define T_WORK     10.0
#define T_RETRACT  3.0
#define T_TOTAL    16.0

/*  (rad) */
#define Q1_WORK   (-30.0 * M_PI / 180.0)
#define Q2_WORK   (-50.0 * M_PI / 180.0)
#define Q3_WORK   (-55.0 * M_PI / 180.0)
#define A1_AMP    ( 30.0 * M_PI / 180.0)
#define OMEGA_B   ( 2.0 * M_PI / T_WORK)

/*  (m) Fluent  */
#define R12_X  0.01669012
#define R12_Y  0.05050000
#define R12_Z -0.03000000
#define R23_X -0.19800000
#define R23_Y  0.05600000
#define R23_Z  0.00189012

/* : c = a x b */
void cross_product(real *a, real *b, real *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/* 3x3  Fluent  */
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

/* 3x3 : y = A * x */
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

/*  () */
void get_joint_trajectory(real t,
                                real *q1,  real *q2,  real *q3,
                                real *qd1, real *qd2, real *qd3)
{
    real u, s, sd, tw;

    u = 0.0;
    s = 0.0;
    sd = 0.0;
    tw = 0.0;

    if (t < 0.0) {
        *q1 = 0.0; *q2 = 0.0; *q3 = 0.0;
        *qd1 = 0.0; *qd2 = 0.0; *qd3 = 0.0;
    }
    else if (t <= T_DEPLOY) {
        /*  A:  (0 ~ 3s) */
        u = t / T_DEPLOY;
        s = 0.5 * (1.0 - cos(M_PI * u));
        sd = 0.5 * (M_PI / T_DEPLOY) * sin(M_PI * u);

        *q1  = Q1_WORK * s;
        *q2  = Q2_WORK * s;
        *q3  = Q3_WORK * s;
        *qd1 = Q1_WORK * sd;
        *qd2 = Q2_WORK * sd;
        *qd3 = Q3_WORK * sd;
    }
    else if (t <= (T_DEPLOY + T_WORK)) {
        /*  B:  (3 ~ 13s) */
        tw = t - T_DEPLOY;
        *q1  = -A1_AMP * cos(OMEGA_B * tw);
        *qd1 =  A1_AMP * OMEGA_B * sin(OMEGA_B * tw);
        *q2  = Q2_WORK;
        *qd2 = 0.0;
        *q3  = Q3_WORK;
        *qd3 = 0.0;
    }
    else if (t <= T_TOTAL) {
        /*  C:  (13 ~ 16s) */
        u = (t - T_DEPLOY - T_WORK) / T_RETRACT;
        s = 0.5 * (1.0 - cos(M_PI * u));
        sd = 0.5 * (M_PI / T_RETRACT) * sin(M_PI * u);
        *q1  = Q1_WORK * (1.0 - s);
        *q2  = Q2_WORK * (1.0 - s);
        *q3  = Q3_WORK * (1.0 - s);
        *qd1 = -Q1_WORK * sd;
        *qd2 = -Q2_WORK * sd;
        *qd3 = -Q3_WORK * sd;
    }
    else {
        /* 16s  */
        *q1 = 0.0; *q2 = 0.0; *q3 = 0.0;
        *qd1 = 0.0; *qd2 = 0.0; *qd3 = 0.0;
    }
}

/*  ( vel  omega) */
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

    get_joint_trajectory(t, &q1, &q2, &q3, &qd1, &qd2, &qd3);

    c1 = cos(q1); s1 = sin(q1);
    c2 = cos(q2); s2 = sin(q2);

    /* 1.  1 ():  Y  */
    w1[0] = 0.0; w1[1] = qd1; w1[2] = 0.0;
    v1[0] = 0.0; v1[1] = 0.0; v1[2] = 0.0;

    if (link_idx == 1) {
        vel[0] = v1[0]; vel[1] = v1[1]; vel[2] = v1[2];
        omega[0] = w1[0]; omega[1] = w1[1]; omega[2] = w1[2];
        return;
    }

    /* 2.  2 () */
    Ry1[0] = c1;  Ry1[1] = 0.0; Ry1[2] = s1;
    Ry1[3] = 0.0; Ry1[4] = 1.0; Ry1[5] = 0.0;
    Ry1[6] = -s1; Ry1[7] = 0.0; Ry1[8] = c1;

    a2[0] = -c1; a2[1] = 0.0; a2[2] = s1;
    w2[0] = w1[0] + qd2 * a2[0];
    w2[1] = w1[1] + qd2 * a2[1];
    w2[2] = w1[2] + qd2 * a2[2];
    r12_0_local[0] = R12_X;
    r12_0_local[1] = R12_Y;
    r12_0_local[2] = R12_Z;
    mat3_vec(Ry1, r12_0_local, r12);
    cross_product(w1, r12, v2);

    if (link_idx == 2) {
        vel[0] = v2[0]; vel[1] = v2[1]; vel[2] = v2[2];
        omega[0] = w2[0]; omega[1] = w2[1]; omega[2] = w2[2];
        return;
    }

    /* 3.  3 () */
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

    r23_2_local[0] = R23_X;
    r23_2_local[1] = R23_Y;
    r23_2_local[2] = R23_Z;
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
 * Fluent DEFINE_CG_MOTION 
 * ========================================================================= */

/*  1 ( lk1) */
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

/*  2 ( lk2) */
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

/*  3 ( lk3w1 ~ lk3w4) */
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
