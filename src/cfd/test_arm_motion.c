#include <stdio.h>
#include <math.h>

#define real double
#define NV_S(a, op, val) a[0] op val; a[1] op val; a[2] op val;
#define DEFINE_CG_MOTION(name, dt, cg_vel, cg_omega, time, dtime) \
    void name(void* dt, real cg_vel[3], real cg_omega[3], real time, real dtime)

#define TEST_TOL 1.0e-12

/*  UDF  arm_motion.c。 */
#define UDF_STANDALONE_TEST
#include "arm_motion.c"

static int near_value(real actual, real expected)
{
    return fabs(actual - expected) <= TEST_TOL;
}

static int check_state(const char *label, real t,
                       real q1, real q2, real q3,
                       real qd1, real qd2, real qd3,
                       real e1, real e2, real e3,
                       real ed1, real ed2, real ed3)
{
    if (near_value(q1, e1) && near_value(q2, e2) && near_value(q3, e3) &&
        near_value(qd1, ed1) && near_value(qd2, ed2) && near_value(qd3, ed3))
        return 1;

    fprintf(stderr, ": %s, t=%.12g s\n", label, t);
    fprintf(stderr, "  q  = [%.15g %.15g %.15g]\n", q1, q2, q3);
    fprintf(stderr, "  qd = [%.15g %.15g %.15g]\n", qd1, qd2, qd3);
    return 0;
}

int main(void)
{
    real q1, q2, q3, qd1, qd2, qd3;
    real tDeploy = ARM_CFG_T_DEPLOY_S;
    real tWorkEnd = ARM_CFG_T_DEPLOY_S + ARM_CFG_T_WORK_S;
    real tTotal = ARM_CFG_T_TOTAL_S;
    real tMid = ARM_CFG_T_DEPLOY_S + 0.5 * ARM_CFG_T_WORK_S;
    real q1Mid = ARM_CFG_Q1_WORK_RAD +
        2.0 * ARM_CFG_Q1_SWEEP_HALF_RAD * ARM_CFG_Q1_SWEEP_DIRECTION;
    real test_t[] = {0.0, 0.5, 1.5, 3.0, 5.5, 8.0, 10.5, 13.0, 14.5, 16.0};
    int n = (int)(sizeof(test_t) / sizeof(test_t[0]));
    int i;

    if (!arm_motion_config_is_valid()) {
        fprintf(stderr, "UDF 。\n");
        return 1;
    }

    get_joint_trajectory(0.0, &q1, &q2, &q3, &qd1, &qd2, &qd3);
    if (!check_state("", 0.0, q1, q2, q3, qd1, qd2, qd3,
                     ARM_CFG_Q1_STOW_RAD, ARM_CFG_Q2_STOW_RAD, ARM_CFG_Q3_STOW_RAD,
                     0.0, 0.0, 0.0)) return 1;

    get_joint_trajectory(tDeploy, &q1, &q2, &q3, &qd1, &qd2, &qd3);
    if (!check_state("", tDeploy, q1, q2, q3, qd1, qd2, qd3,
                     ARM_CFG_Q1_WORK_RAD, ARM_CFG_Q2_WORK_RAD, ARM_CFG_Q3_WORK_RAD,
                     0.0, 0.0, 0.0)) return 1;

    get_joint_trajectory(tMid, &q1, &q2, &q3, &qd1, &qd2, &qd3);
    if (!check_state("", tMid, q1, q2, q3, qd1, qd2, qd3,
                     q1Mid, ARM_CFG_Q2_WORK_RAD, ARM_CFG_Q3_WORK_RAD,
                     0.0, 0.0, 0.0)) return 1;

    get_joint_trajectory(tWorkEnd, &q1, &q2, &q3, &qd1, &qd2, &qd3);
    if (!check_state("", tWorkEnd, q1, q2, q3, qd1, qd2, qd3,
                     ARM_CFG_Q1_WORK_RAD, ARM_CFG_Q2_WORK_RAD, ARM_CFG_Q3_WORK_RAD,
                     0.0, 0.0, 0.0)) return 1;

    get_joint_trajectory(tTotal, &q1, &q2, &q3, &qd1, &qd2, &qd3);
    if (!check_state("", tTotal, q1, q2, q3, qd1, qd2, qd3,
                     ARM_CFG_Q1_STOW_RAD, ARM_CFG_Q2_STOW_RAD, ARM_CFG_Q3_STOW_RAD,
                     0.0, 0.0, 0.0)) return 1;

    get_joint_trajectory(tTotal + 1.0, &q1, &q2, &q3, &qd1, &qd2, &qd3);
    if (!check_state("", tTotal + 1.0, q1, q2, q3, qd1, qd2, qd3,
                     ARM_CFG_Q1_STOW_RAD, ARM_CFG_Q2_STOW_RAD, ARM_CFG_Q3_STOW_RAD,
                     0.0, 0.0, 0.0)) return 1;

    printf("===  C UDF  ===\n");
    printf("=%.6g s, =%.6g s, =%.6g deg\n",
           (double)ARM_CFG_T_TOTAL_S, (double)ARM_CFG_TIME_STEP_S,
           (double)ARM_CFG_Q1_SWEEP_HALF_DEG);
    for (i = 0; i < n; i++) {
        real t = test_t[i];
        real v1[3], w1[3], v2[3], w2[3], v3[3], w3[3];
        calc_link_motion(t, 1, v1, w1);
        calc_link_motion(t, 2, v2, w2);
        calc_link_motion(t, 3, v3, w3);
        printf("t=%4.1fs | w1=[% .6f % .6f % .6f] | "
               "w2=[% .6f % .6f % .6f] | w3=[% .6f % .6f % .6f]\n",
               t, w1[0], w1[1], w1[2], w2[0], w2[1], w2[2],
               w3[0], w3[1], w3[2]);
    }
    printf(" UDF 。\n");
    return 0;
}
