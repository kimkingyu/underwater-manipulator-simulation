function diag_case(case_id)
%DIAG_CASE  CFD vs theta prediction (current fit) for one case, all 27 channels, with the per-group
%   data-set RMS used for normalisation.
if nargin < 1, case_id = 'T_px_08'; end
here = fileparts(mfilename('fullpath'));
o = fit_theta(fullfile(here, '..', 'theta_dataset.json'), tempdir, 'lin', true);
S = jsondecode(fileread(fullfile(here, '..', 'theta_dataset.json')));
C = S.cases; if isstruct(C), C = num2cell(C); end
G = uvms_geometry();
k = C{find(cellfun(@(c) strcmp(c.id, case_id), C), 1)};
nu = reshape(k.nu, [], 6); nd = reshape(k.nudot, [], 6);
L = reshape(k.flink, [], 18);
[Yg, Yl] = Y_uvms(zeros(3, 1), [nu(1, :)'; 0; 0; 0], [nd(1, :)'; 0; 0; 0], G);
cfd = [k.F(1, :) k.M(1, :) k.tau(1, :) L(1, :)];
pred = -[Yg * o.theta; Yl * o.theta]';
lab = {'Fx','Fy','Fz','Mx','My','Mz','tau1','tau2','tau3', ...
       'L1fx','L1fy','L1fz','L1mx','L1my','L1mz','L2fx','L2fy','L2fz','L2mx','L2my','L2mz', ...
       'L3fx','L3fy','L3fz','L3mx','L3my','L3mz'};
fprintf('%s   channel     CFD      theta     scale\n', case_id);
for i = 1:27
    fprintf('  %-6s %9.4f %9.4f %9.4f\n', lab{i}, cfd(i), pred(i), o.channel_scale(i));
end
end
