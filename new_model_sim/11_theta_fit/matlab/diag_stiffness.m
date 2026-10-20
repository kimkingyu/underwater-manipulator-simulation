function diag_stiffness()
%DIAG_STIFFNESS  Closed-loop PD eigenvalues at q = 0 with each hydro model's mass matrix, and the
%   RK4 amplification at dt = 0.005 s (> 1 means the explicit integration is unstable).
here = fileparts(mfilename('fullpath'));
G = uvms_geometry();
S = load(fullfile(here, 'theta_fit.mat')); th = S.theta;
z = zeros(9, 1);
MA = zeros(9);
for j = 1:9
    e = z; e(j) = 1;
    MA(:, j) = uvms_hydro_theta(z, z, e, th, G)';
end
MA = (MA + MA') / 2;
u = build_chain(fullfile(here, '..', '..', '..', 'model', 'robot.urdf'));
Mr = massMatrix(u, z);
m = 127.84851;
HS = zeros(9);
HS(1:3, 1:3) = diag([0.4 0.4 0.5] * m); HS(4:6, 4:6) = diag([6 6 6]);
HS(7, 7) = 0.05; HS(8, 8) = 0.15; HS(9, 9) = 0.12;
Kp = diag([350 350 450 120 120 100 120 100 80]);
Kd = diag([180 180 220 60 60 50 25 20 15]);
dt = 0.005;
names = {'handset', 'theta'}; Ms = {Mr + HS, Mr + MA};
for c = 1:2
    M = Ms{c};
    A = [zeros(9) eye(9); -(M \ Kp) -(M \ Kd)];
    ev = eig(A);
    lam = ev * dt;
    amp = abs(1 + lam + lam .^ 2 / 2 + lam .^ 3 / 6 + lam .^ 4 / 24);
    Mi = inv(M);
    fprintf('%-8s  fastest |eig| = %8.1f 1/s   max RK4 amplification = %.3f   effective J1/J2/J3 inertia = %.4f %.4f %.4f\n', ...
        names{c}, max(abs(ev)), max(amp), 1 / Mi(7, 7), 1 / Mi(8, 8), 1 / Mi(9, 9));
end
fprintf('\nrow J1 of rigid M:  %s\n', mat2str(Mr(7, :), 3));
fprintf('row J1 of theta MA: %s\n', mat2str(MA(7, :), 3));
fprintf('row J1 of handset:  %s\n', mat2str(HS(7, :), 3));
end
