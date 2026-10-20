% inspect_mat.m
rootDir = 'D:/work4';
s = load(fullfile(rootDir, 'data', 'joint_loads_data.mat'));
disp(fieldnames(s));
if isfield(s, 'traj')
    disp('traj fields:');
    disp(fieldnames(s.traj));
end
