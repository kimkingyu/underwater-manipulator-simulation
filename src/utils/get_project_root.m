function rootDir = get_project_root()
% GET_PROJECT_ROOT 返回水下机械臂仿真工程的根目录绝对路径
% 无论当前工作目录处于根目录、src、experiments 还是 docs，均能自动准确解析定位

currentFile = mfilename('fullpath');
utilsDir = fileparts(currentFile);       % .../src/utils
srcDir   = fileparts(utilsDir);          % .../src
rootDir  = fileparts(srcDir);           % .../

% 安全回退检查
if ~isfile(fullfile(rootDir, 'model', 'robot.urdf'))
    % 尝试以当前工作目录探测
    if isfile(fullfile(pwd, 'model', 'robot.urdf'))
        rootDir = pwd;
    end
end
end
