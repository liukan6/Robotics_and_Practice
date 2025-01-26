clear;clc;
% 定义theta总值矩阵
thetaTotalList = [pi/6, 0, pi/6, 0, pi/3, 0;
                  pi/6, pi/6, pi/3, 0, pi/3, pi/6;
                  pi/2, 0, pi/2, -pi/3, pi/3, pi/6;
                  -pi/6, -pi/6, -pi/3, 0, pi/12, pi/2;
                  pi/12, pi/12, pi/12, pi/12, pi/12, pi/12];
% 定义具体的theta初始值和总值矩阵
initThetaList = [0, -pi/2, 0, pi/2, pi/2, 0];
thetaList = thetaTotalList(1, :) + initThetaList;
% 定义DH参数
aList = [0, 185, 170, 0, 0, 0]*0.001;
dList = [230, 0, 0, 23, 77, 85.5]*0.001;
alphaList = [-pi/2, 0, 0, pi/2, pi/2, 0];
% 计算每个关节的变换矩阵
T1 = transform(dList(1), thetaList(1), aList(1), alphaList(1));
T2 = transform(dList(2), thetaList(2), aList(2), alphaList(2));
T3 = transform(dList(3), thetaList(3), aList(3), alphaList(3));
T4 = transform(dList(4), thetaList(4), aList(4), alphaList(4));
T5 = transform(dList(5), thetaList(5), aList(5), alphaList(5));
T6 = transform(dList(6), thetaList(6), aList(6), alphaList(6));
% 总的齐次变换矩阵
Ttotal = T1 * T2 * T3 * T4 * T5 * T6;
% 输出齐次变换矩阵
disp('齐次变换矩阵 T:');disp(Ttotal);
% 提取末端执行器的位置坐标
pos = Ttotal * [0;0;0;1];
disp(['末端位置：x = ', num2str(pos(1)), ',' ...
    ' y = ', num2str(pos(2)), ', z = ', num2str(pos(3))]);
% 计算欧拉角
beta = atan2(Ttotal(1,3), sqrt(Ttotal(1,1)^2 + Ttotal(1,2)^2));
alpha = atan2(-Ttotal(2,3)/cos(beta), Ttotal(3,3)/cos(beta));
gamma = atan2(-Ttotal(1,2)/cos(beta), Ttotal(1,1)/cos(beta));

fprintf('欧拉角 alpha: %f 度\n', rad2deg(alpha));
fprintf('欧拉角 beta: %f 度\n', rad2deg(beta));
fprintf('欧拉角 gamma: %f 度\n', rad2deg(gamma));
% DH参数转齐次变换矩阵的函数
function T = transform(d, theta, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),             cos(alpha),            d;
         0,          0,                      0,                     1];
end