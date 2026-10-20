%EXAMPLE_Y_UVMS  How to use the fitted theta in the controller.
%   tau_hat = Y_uvms(q, zeta, a) * theta  is the estimate of  M_A a + C_A(q,zeta) zeta + D_0 + tau_f0
%   (generalised force, 9x1: AUV force [N] and moment about O_b [N*m] in the body frame, then J1..J3 [N*m]).
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'theta_fit.mat'));
theta = S.theta;                       % 60 x 1, see theta_names / theta_table.csv
G = uvms_geometry();

q    = [0.2; -0.3; 0.4];               % joint angles [rad]
zeta = [0.5; 0.1; -0.05; 0; 0; 0.1; 0.3; -0.2; 0.1];   % [u v w p q r qd1 qd2 qd3]
a    = [0.2; 0; 0; 0; 0; 0.05; 0.5; 0; 0];            % reference acceleration

Y = Y_uvms(q, zeta, a, G);             % 9 x 60
tau_hat = Y * theta;
names = {'X', 'Y', 'Z', 'K', 'M', 'N', 'tau1', 'tau2', 'tau3'};
for i = 1:9
    fprintf('%-5s %10.4f\n', names{i}, tau_hat(i));
end
% Friction theta(37:42) is 0 here (not identifiable from CFD); an adaptive law can update it online.
