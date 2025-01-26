% 定义机器人模型
robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', 6);

% 创建关节和连杆，并定义DH参数
body1 = rigidBody('body1');
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1, dhparams(0, 0.23, 0, -pi/2)); % 使用 DH 参数
body1.Joint = joint1;
addBody(robot, body1, 'base');

body2 = rigidBody('body2');
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2, dhparams(-pi/2, 0, 0.185, 0));
body2.Joint = joint2;
addBody(robot, body2, 'body1');

body3 = rigidBody('body3');
joint3 = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(joint3, dhparams(0, 0, 0.17, 0));
body3.Joint = joint3;
addBody(robot, body3, 'body2');

body4 = rigidBody('body4');
joint4 = rigidBodyJoint('joint4', 'revolute');
setFixedTransform(joint4, dhparams(pi/2, 0.023, 0, pi/2));
body4.Joint = joint4;
addBody(robot, body4, 'body3');

body5 = rigidBody('body5');
joint5 = rigidBodyJoint('joint5', 'revolute');
setFixedTransform(joint5, dhparams(pi/2, 0.077, 0, pi/2));
body5.Joint = joint5;
addBody(robot, body5, 'body4');

body6 = rigidBody('body6');
joint6 = rigidBodyJoint('joint6', 'revolute');
setFixedTransform(joint6, dhparams(0, 0.0855, 0, 0));
body6.Joint = joint6;
addBody(robot, body6, 'body5');

% 定义逆运动学求解器
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1]; % 权重，用于姿态优先级
initialguess = robot.homeConfiguration; % 初始猜测

% 定义五组目标末端位姿 (坐标 + 旋转角度)
joint_angles = [
    0.117, 0.334, 0.499, -2.019, -0.058, -2.190;  % 第一组
    -0.066, 0.339, 0.444, -2.618, -0.524, -3.141; % 第二组
    0.3, 0.25, 0.26, -2.64, 0.59, -2.35;          % 第三组
    0.42, 0, 0.36, 3.14, 1, -1.57;                % 第四组
    0.32, -0.25, 0.16, 3, 0.265, -0.84            % 第五组
];

% 求解逆运动学
for i = 1:size(joint_angles, 1)
    disp(['计算第 ', num2str(i), ' 组参数的逆运动学解:']);
    
    % 提取当前的末端位置和旋转角度
    position = joint_angles(i, 1:3);  % 前三列是坐标 (X, Y, Z)
    rotation = joint_angles(i, 4:6);  % 后三列是旋转角度 (Rx, Ry, Rz)
    
    % 将欧拉角转换为旋转矩阵
    R = eul2rotm(rotation, 'XYZ'); % XYZ顺序
    
    % 创建目标位姿的齐次变换矩阵
    T = trvec2tform(position) * rotm2tform(R);
    
    % 求解逆运动学
    [configSol, solInfo] = ik('body6', T, weights, initialguess);
    
    % 检查求解信息
    if solInfo.Status == "success"
        % 显示求解的关节角度
        disp(['theta1 = ', num2str(configSol(1))]);
        disp(['theta2 = ', num2str(configSol(2))]);
        disp(['theta3 = ', num2str(configSol(3))]);
        disp(['theta4 = ', num2str(configSol(4))]);
        disp(['theta5 = ', num2str(configSol(5))]);
        disp(['theta6 = ', num2str(configSol(6))]);
        fprintf('[%f, %f, %f, %f, %f, %f]\n', configSol(1), configSol(2), configSol(3), configSol(4), configSol(5), configSol(6));
    else
        disp('求解失败！');
    end
end

function T = dhparams(theta, d, a, alpha)
    % 生成基于DH参数的齐次变换矩阵
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end
