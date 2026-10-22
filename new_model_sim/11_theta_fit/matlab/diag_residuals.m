function diag_residuals()
%DIAG_RESIDUALS  Channel-by-channel CFD vs theta (theta_fit.mat, full fit) for the steady q = 0 cases:
%   F, M (about O_b), tau. Prints CFD, prediction and residual, and for each channel the share of the
%   total squared residual (weighted as in fit_theta) -- where the remaining error lives.
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'theta_fit.mat'));
D = jsondecode(fileread(fullfile(here, '..', 'theta_dataset.json')));
C = D.cases; if isstruct(C), C = num2cell(C); end
G = uvms_geometry();
nm = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 't1', 't2', 't3'};
grp = [1 1 1 2 2 2 3 3 3];
sc = S.channel_scale(1:9);
tot = zeros(1, 9); n = 0;
fprintf('%-9s %s\n', 'case', strjoin(cellfun(@(x) sprintf('%22s', x), nm, 'UniformOutput', false), ''));
for c = 1:numel(C)
    k = C{c};
    if ~startsWith(k.id, 'T_'), continue; end
    nu = k.nu(:); F = k.F(:); M = k.M(:); T = k.tau(:);
    y = [F; M; T];
    Yg = Y_uvms(zeros(3, 1), [nu; 0; 0; 0], zeros(9, 1), G);
    p = -Yg * S.theta;
    r = p - y;
    tot = tot + (r' ./ sc') .^ 2; n = n + 1;
    fprintf('%-9s', k.id);
    for j = 1:9, fprintf(' %7.2f/%6.2f/%+6.2f', y(j), p(j), r(j)); end
    fprintf('\n');
end
fprintf('\nshare of the weighted squared residual over the steady cases:\n');
share = tot / sum(tot);
for j = 1:9, fprintf('  %s %.1f%%', nm{j}, 100 * share(j)); end
fprintf('\n(CFD / prediction / residual; F in N, M and tau in N m)\n');
end
