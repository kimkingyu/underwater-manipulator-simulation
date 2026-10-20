function [R_bc, t_bc] = urdf_to_cad()
%URDF_TO_CAD  Rigid map from the old model/robot.urdf base_link frame to the new AUVM CAD frame:
%   x_cad = R_bc * x_bl + t_bc.
%   Fitted on the three URDF joint centres (base_link, q = 0) against their positions in the new CAD
%   (02_ansys_read_check/joint_candidates_from_old_urdf.json): position error 3.5e-17 m, and R_bc maps
%   the URDF joint axes exactly onto the CFD joint axes, so q and joint torques have the same sign.
R_bc = [1 0 0; 0 0 1; 0 -1 0];
t_bc = [-4e-06; 0.35; -0.052691];
end
