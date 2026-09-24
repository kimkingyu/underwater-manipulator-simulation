function cfg = cfd_case_parameters()
%CFD_CASE_PARAMETERS Single user-editable Fluent CFD case configuration.
% Edit values in this file, then run prepare_fluent_udf.
% Units: angles [deg], time [s], lengths [m], speed [m/s].
%
% The geometry block is intentionally fixed to the validated CAD/mesh
% coordinate system. Change it only when the mesh or CAD model changes.

cfg = struct();
cfg.schemaVersion = 1;
cfg.caseId = 'nominal_three_phase';

cfg.motion = struct();
cfg.motion.qStowDeg = [0.0, 0.0, 0.0];
cfg.motion.qWorkDeg = [-30.0, -50.0, -55.0];
cfg.motion.durationsS = [2.0, 6.0, 2.0];
cfg.motion.q1SweepHalfDeg = 30.0;
cfg.motion.q1SweepDirection = 1.0;

cfg.solver = struct();
cfg.solver.timeStepS = 0.01;
cfg.solver.previewSteps = 30;
cfg.solver.totalTimeS = sum(cfg.motion.durationsS);
cfg.solver.fullSteps = round(cfg.solver.totalTimeS / cfg.solver.timeStepS);

cfg.environment = struct();
cfg.environment.densityKgM3 = 1025.0;
cfg.environment.gravityMS2 = 9.81;
cfg.environment.dynamicViscosityPaS = 1.05e-3;
cfg.environment.currentSpeedMPS = 0.25;
cfg.environment.currentAzimuthDeg = 45.0;
cfg.environment.currentElevationDeg = 0.0;

cfg.geometry = struct();
cfg.geometry.r12M = [0.01669012, 0.05050000, -0.03000000];
cfg.geometry.r23M = [-0.19800000, 0.05600000, 0.00189012];
cfg.geometry.jointCentersM = [ ...
    -0.745634, 1.155872, -1.142240; ...
    -0.728944, 1.206372, -1.172240; ...
    -0.730834, 1.008372, -1.228240];
cfg.geometry.jointAxesInitial = [0, 1, 0; -1, 0, 0; 1, 0, 0];
cfg.geometry.zoneNames = {'lk1','lk2','lk3w1','lk3w2','lk3w3','lk3w4'};
cfg.geometry.motionUDFs = {'joint1_motion','joint2_motion','joint3_motion', ...
    'joint3_motion','joint3_motion','joint3_motion'};

cfg.output = struct();
cfg.output.reportFileStem = 'fluent_joint_moment';
cfg.output.caseFileStem = 'FFF_motion_ready_t0';
cfg.output.signConvention = 'fluid_on_arm_positive_URDF_joint_axis';
end
