function [free, info] = uvms_joint_check(q)
%UVMS_JOINT_CHECK  Collision check of arm poses against the new-CAD scan (13_pose_q1/joint_limits.json).
%   q: n x 3 joint angles [rad]. free(k) is true when pose k is collision-free.
%   Scan (sc_joint_limits.py, AUVM.step parts, hit = overlap > 0.5 mm3 between different link groups):
%     single joints from q = 0: J1 free over [-1.57, 1.57]; J2 and J3 hit at +0.005 rad (q = 0 is the
%     positive mechanical stop of both), free down to -1.57.
%     J2 x J3 grids at J1 = -1.57, -0.785, 0, 0.785, 1.57 (step 0.157, 0.0785 at J1 = 0).
%   A pose is free when it is inside the box [-1.57 1.57] x [-1.57 0] x [-1.57 0] (tolerance 1e-9) and
%   all four grid nodes around (q2, q3) are free on the nearest scanned J1 level -- conservative between
%   nodes. Beyond |q| = 1.57 nothing was scanned: reported as not free.
persistent S
if isempty(S)
    here = fileparts(mfilename('fullpath'));
    J = jsondecode(fileread(fullfile(here, '..', '..', '13_pose_q1', 'joint_limits.json')));
    fn = fieldnames(J); g = fn(startsWith(fn, 'grid_j1'));
    S.lv = zeros(numel(g), 1); S.g = cell(numel(g), 1);
    for i = 1:numel(g)
        x = J.(g{i});
        S.lv(i) = x.q1;
        S.g{i} = struct('q2', x.q2_values(:), 'q3', x.q3_values(:), 'free', double(x.free) == 1);
    end
    [S.lv, o] = sort(S.lv); S.g = S.g(o);
    S.box = [-1.57 1.57; -1.57 0; -1.57 0];
end
q = reshape(q, [], 3);
n = size(q, 1);
free = false(n, 1);
for k = 1:n
    inbox = all(q(k, :)' >= S.box(:, 1) - 1e-9 & q(k, :)' <= S.box(:, 2) + 1e-9);
    if ~inbox, continue; end
    [~, i] = min(abs(S.lv - q(k, 1)));
    g = S.g{i};
    a = nodes(g.q2, q(k, 2)); b = nodes(g.q3, q(k, 3));
    free(k) = all(all(g.free(a, b)));
end
info = struct('box', S.box, 'j1_levels', S.lv);
end

function idx = nodes(v, x)
% indices of the grid nodes bracketing x (one node when x is on a node)
d = v - x;
on = find(abs(d) < 1e-9, 1);
if ~isempty(on), idx = on; return; end
lo = find(d < 0, 1, 'last'); hi = find(d > 0, 1, 'first');
idx = [lo; hi];
end
