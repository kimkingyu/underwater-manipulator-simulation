function uvms = build_chain(urdfFile)
%BUILD_CHAIN  The 9-DOF floating-base chain of experiments/simulate_uvms_dynamics.m (same construction):
%   prismatic x, y, z, revolute roll (x), pitch (y), yaw (z) carrying base_link ('auv_base'), then the arm.
armRobot = importrobot(urdfFile);
armRobot.DataFormat = 'column';
uvms = rigidBodyTree('DataFormat', 'column');
uvms.Gravity = [0 0 -9.81];
jointTypes = {'prismatic', 'prismatic', 'prismatic', 'revolute', 'revolute', 'revolute'};
jointAxes = [1 0 0; 0 1 0; 0 0 1; 1 0 0; 0 1 0; 0 0 1];
parentNames = {uvms.BaseName, 'virtual_x', 'virtual_y', 'virtual_z', 'virtual_roll', 'virtual_pitch'};
bodyNames = {'virtual_x', 'virtual_y', 'virtual_z', 'virtual_roll', 'virtual_pitch'};
for i = 1:5
    b = rigidBody(bodyNames{i});
    j = rigidBodyJoint(['joint_' bodyNames{i}], jointTypes{i});
    j.JointAxis = jointAxes(i, :);
    b.Joint = j; b.Mass = 0; b.CenterOfMass = [0 0 0]; b.Inertia = zeros(1, 6);
    addBody(uvms, b, parentNames{i});
end
auvBody = copy(armRobot.Base); auvBody.Name = 'auv_base';
jAuv = rigidBodyJoint('joint_virtual_yaw', 'revolute'); jAuv.JointAxis = jointAxes(6, :);
auvBody.Joint = jAuv;
addBody(uvms, auvBody, 'virtual_pitch');
for i = 1:armRobot.NumBodies
    src = armRobot.Bodies{i}; nb = copy(src); par = src.Parent.Name;
    if strcmp(par, armRobot.BaseName), par = 'auv_base'; end
    addBody(uvms, nb, par);
end
end
