function report = extract_urdf_hydro_geometry()
% 从 URDF 的惯性参数和 collision STL 网格提取水动力建模的几何基础数据。
% 输出：各连杆质量、质心、惯量、封闭网格体积、几何浮心、表面积、投影面积近似值。
% 说明：网格体积只在 STL 封闭且对应腔体不进水时，才能直接作为排水体积。

scriptDir = fileparts(mfilename('fullpath'));
urdfFile = fullfile(scriptDir, 'robot.urdf');
rhoWater = 1025;      % 默认海水密度 [kg/m^3]
g = 9.81;             % 重力加速度 [m/s^2]

if ~isfile(urdfFile)
    error('未找到 URDF 文件：%s', urdfFile);
end

xmlDoc = xmlread(urdfFile);
linkNodes = xmlDoc.getElementsByTagName('link');
numLinks = linkNodes.getLength;

linkName = strings(numLinks, 1);
mass_kg = zeros(numLinks, 1);
com_m = zeros(numLinks, 3);
inertia_kgm2 = zeros(numLinks, 6); % [Ixx Iyy Izz Ixy Ixz Iyz]
volume_m3 = zeros(numLinks, 1);
cb_m = nan(numLinks, 3);
surfaceArea_m2 = zeros(numLinks, 1);
projectedAreaXYZ_m2 = zeros(numLinks, 3);
meshCount = zeros(numLinks, 1);
allMeshesWatertight = false(numLinks, 1);
meshBounds_m = nan(numLinks, 6);  % [xmin xmax ymin ymax zmin zmax]

for linkIndex = 1:numLinks
    linkNode = linkNodes.item(linkIndex - 1);
    linkName(linkIndex) = string(char(linkNode.getAttribute('name')));

    inertialNode = linkNode.getElementsByTagName('inertial').item(0);
    inertialOrigin = inertialNode.getElementsByTagName('origin').item(0);
    massNode = inertialNode.getElementsByTagName('mass').item(0);
    inertiaNode = inertialNode.getElementsByTagName('inertia').item(0);

    com_m(linkIndex, :) = getVectorAttribute(inertialOrigin, 'xyz', [0 0 0]);
    mass_kg(linkIndex) = str2double(char(massNode.getAttribute('value')));
    inertia_kgm2(linkIndex, :) = [ ...
        str2double(char(inertiaNode.getAttribute('ixx'))), ...
        str2double(char(inertiaNode.getAttribute('iyy'))), ...
        str2double(char(inertiaNode.getAttribute('izz'))), ...
        str2double(char(inertiaNode.getAttribute('ixy'))), ...
        str2double(char(inertiaNode.getAttribute('ixz'))), ...
        str2double(char(inertiaNode.getAttribute('iyz')))];

    collisionNodes = linkNode.getElementsByTagName('collision');
    meshCount(linkIndex) = collisionNodes.getLength;
    allMeshesWatertight(linkIndex) = meshCount(linkIndex) > 0;

    linkVolume = 0;
    weightedCentroid = [0 0 0];
    linkSurfaceArea = 0;
    linkProjectedArea = [0 0 0];
    linkMin = [inf inf inf];
    linkMax = [-inf -inf -inf];

    for meshIndex = 1:meshCount(linkIndex)
        collisionNode = collisionNodes.item(meshIndex - 1);
        meshNode = collisionNode.getElementsByTagName('mesh').item(0);
        originNode = collisionNode.getElementsByTagName('origin').item(0);

        meshFile = fullfile(scriptDir, char(meshNode.getAttribute('filename')));
        if ~isfile(meshFile)
            error('未找到网格文件：%s', meshFile);
        end

        scale = getVectorAttribute(meshNode, 'scale', [1 1 1]);
        translation = getVectorAttribute(originNode, 'xyz', [0 0 0]);
        rpy = getVectorAttribute(originNode, 'rpy', [0 0 0]);
        R = rpyToRotationMatrix(rpy);

        tri = stlread(meshFile);
        vertices = tri.Points .* scale;
        vertices = (R * vertices.').';
        vertices = vertices + translation;
        faces = tri.ConnectivityList;

        [signedVolume, meshCentroid, meshArea, projectionArea, isWatertight] = ...
            meshProperties(vertices, faces);
        absoluteVolume = abs(signedVolume);

        linkVolume = linkVolume + absoluteVolume;
        weightedCentroid = weightedCentroid + absoluteVolume * meshCentroid;
        linkSurfaceArea = linkSurfaceArea + meshArea;
        linkProjectedArea = linkProjectedArea + projectionArea;
        linkMin = min(linkMin, min(vertices, [], 1));
        linkMax = max(linkMax, max(vertices, [], 1));
        allMeshesWatertight(linkIndex) = allMeshesWatertight(linkIndex) && isWatertight;
    end

    volume_m3(linkIndex) = linkVolume;
    surfaceArea_m2(linkIndex) = linkSurfaceArea;
    projectedAreaXYZ_m2(linkIndex, :) = linkProjectedArea;
    meshBounds_m(linkIndex, :) = [linkMin(1), linkMax(1), ...
                                  linkMin(2), linkMax(2), ...
                                  linkMin(3), linkMax(3)];
    if linkVolume > eps
        cb_m(linkIndex, :) = weightedCentroid / linkVolume;
    end
end

buoyancy_N = rhoWater * g * volume_m3;
weight_N = mass_kg * g;
netVerticalForce_N = buoyancy_N - weight_N;
neutralVolume_m3 = mass_kg / rhoWater;

linkTable = table(linkName, mass_kg, ...
    com_m(:, 1), com_m(:, 2), com_m(:, 3), ...
    inertia_kgm2(:, 1), inertia_kgm2(:, 2), inertia_kgm2(:, 3), ...
    inertia_kgm2(:, 4), inertia_kgm2(:, 5), inertia_kgm2(:, 6), ...
    volume_m3, neutralVolume_m3, ...
    cb_m(:, 1), cb_m(:, 2), cb_m(:, 3), ...
    surfaceArea_m2, ...
    projectedAreaXYZ_m2(:, 1), projectedAreaXYZ_m2(:, 2), projectedAreaXYZ_m2(:, 3), ...
    buoyancy_N, weight_N, netVerticalForce_N, allMeshesWatertight, ...
    'VariableNames', { ...
    'Link', 'Mass_kg', 'COM_X_m', 'COM_Y_m', 'COM_Z_m', ...
    'Ixx_kgm2', 'Iyy_kgm2', 'Izz_kgm2', 'Ixy_kgm2', 'Ixz_kgm2', 'Iyz_kgm2', ...
    'MeshVolume_m3', 'NeutralVolume_m3', 'CB_X_m', 'CB_Y_m', 'CB_Z_m', ...
    'SurfaceArea_m2', 'ProjectedArea_X_m2', 'ProjectedArea_Y_m2', 'ProjectedArea_Z_m2', ...
    'Buoyancy_N', 'Weight_N', 'NetUpwardForce_N', 'AllMeshesWatertight'});

report = struct();
report.waterDensity_kg_m3 = rhoWater;
report.gravity_m_s2 = g;
report.links = linkTable;
report.totalMass_kg = sum(mass_kg);
report.totalMeshVolume_m3 = sum(volume_m3);
report.totalNeutralVolume_m3 = report.totalMass_kg / rhoWater;
report.totalBuoyancy_N = sum(buoyancy_N);
report.totalWeight_N = sum(weight_N);
report.totalNetUpwardForce_N = sum(netVerticalForce_N);
report.note = ['MeshVolume_m3 和 CB 仅可在所有网格封闭、彼此不重叠、' ...
    '且结构内部不进水时作为真实排水体积与浮心使用。' ...
    'ProjectedArea_*_m2 为基于三角面法向的阻力面积近似。'];

save(fullfile(scriptDir, 'urdf_hydro_geometry.mat'), 'report');
writetable(linkTable, fullfile(scriptDir, 'urdf_hydro_geometry.csv'));

disp(linkTable(:, {'Link', 'Mass_kg', 'MeshVolume_m3', 'NeutralVolume_m3', ...
    'Buoyancy_N', 'Weight_N', 'NetUpwardForce_N', 'AllMeshesWatertight'}));
fprintf('\n总质量：%.6f kg\n', report.totalMass_kg);
fprintf('CAD 网格体积合计：%.9f m^3\n', report.totalMeshVolume_m3);
fprintf('中性浮力所需体积：%.9f m^3\n', report.totalNeutralVolume_m3);
fprintf('静态合力（向上为正）：%.3f N\n', report.totalNetUpwardForce_N);
end

function value = getVectorAttribute(node, attributeName, defaultValue)
raw = char(node.getAttribute(attributeName));
if isempty(raw)
    value = defaultValue;
    return;
end
value = sscanf(raw, '%f').';
if numel(value) ~= numel(defaultValue)
    error('属性 %s 的数值数量不正确。', attributeName);
end
end

function R = rpyToRotationMatrix(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
R = Rz * Ry * Rx;
end

function [signedVolume, centroid, surfaceArea, projectedArea, isWatertight] = meshProperties(vertices, faces)
p1 = vertices(faces(:, 1), :);
p2 = vertices(faces(:, 2), :);
p3 = vertices(faces(:, 3), :);

crossProduct = cross(p2 - p1, p3 - p1, 2);
doubleArea = vecnorm(crossProduct, 2, 2);
triangleArea = 0.5 * doubleArea;
surfaceArea = sum(triangleArea);

normals = crossProduct ./ max(doubleArea, eps);
projectedArea = 0.5 * sum(abs(normals) .* triangleArea, 1);

sixTimesVolume = dot(p1, cross(p2, p3, 2), 2);
signedVolume = sum(sixTimesVolume) / 6;
if abs(signedVolume) < eps
    centroid = [nan nan nan];
else
    centroid = sum((p1 + p2 + p3) .* sixTimesVolume, 1) / (24 * signedVolume);
end

edges = sort([faces(:, [1 2]); faces(:, [2 3]); faces(:, [3 1])], 2);
[~, ~, edgeIndex] = unique(edges, 'rows');
edgeCounts = accumarray(edgeIndex, 1);
isWatertight = all(edgeCounts == 2);
end
