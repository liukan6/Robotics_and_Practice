% 约束条件
% 初始条件 (位置、速度)
X1 = 1;Vx0 = 0; 
Y0 = 0;  
Yf1 = 1; 
Yf2 = 2; 
Z0 = 0;  
Vz0 = 0; 
Zf1 = -1;
Zf2 = -2;

%% 方程1: X1(t), Y1(t), Z1(t) 为4次多项式

% 对X1(t)的约束 (4次方)
A_X1 = [
    1 0 0 0 0;   % X1(0) = X0
    0 1 0 0 0;   % V1(0) = V0
    1 1 1 1 1;   % X1(1) = Xf1
    0 1 2 3 4    % V1(1) = Vf1
];
B_X1 = [X1; Vx0; -1; 0]; 

% 对Y1(t)的约束 (4次方)
A_Y1 = [
    1 0 0 0 0;   % Y1(0) = Y0
    0 1 0 0 0;   % V1(0) = 0
    1 1 1 1 1;   % Y1(1) = Yf1
    0 1 2 3 4    % V1(1) = 0
];
B_Y1 = [Y0; 0; Yf1; 0];

% 对Z1(t)的约束 (4次方)
A_Z1 = [
    1 0 0 0 0;   % Z1(0) = Z0
    0 1 0 0 0;   % Vz1(0) = Vz0
    1 1 1 1 1;   % Z1(1) = Zf1
    0 1 2 3 4    % Vz1(1) = 0
];
B_Z1 = [Z0; Vz0; Zf1; 0];

% 解多项式系数
coeff_X1 = A_X1 \ B_X1;  
coeff_Y1 = A_Y1 \ B_Y1;  
coeff_Z1 = A_Z1 \ B_Z1;  

%% 方程2: X2(t), Y2(t), Z2(t) 为1次多项式

% 对X2(t)的约束 (1次方)
A_X2 = [
    1 0;   % X2(0) = Xf1
    1 1    % X2(1) = Xf2
];
B_X2 = [Xf1; -1];  % X2的终止位置

% 对Y2(t)的约束 (1次方)
A_Y2 = [
    1 0;   % Y2(0) = Yf1
    1 1    % Y2(1) = Yf2
];
B_Y2 = [Yf1; Yf2];

% 对Z2(t)的约束 (1次方)
A_Z2 = [
    1 0;   % Z2(0) = Zf1
    1 1    % Z2(1) = Zf2
];
B_Z2 = [Zf1; Zf2];

% 解多项式系数
coeff_X2 = A_X2 \ B_X2;  
coeff_Y2 = A_Y2 \ B_Y2;  
coeff_Z2 = A_Z2 \ B_Z2;  

%% 方程3: X3(t), Y3(t), Z3(t) 为4次多项式

% 对X3(t)的约束 (4次方)
A_X3 = [
    1 0 0 0 0;   % X3(0) = Xf2
    0 1 0 0 0;   % V3(0) = 0
    1 1 1 1 1;   % X3(1) = Xf3
    0 1 2 3 4    % V3(1) = 0
];
B_X3 = [Xf1; 0; -1; 0]; 

% 对Y3(t)的约束 (4次方)
A_Y3 = [
    1 0 0 0 0;   % Y3(0) = Yf2
    0 1 0 0 0;   % V3(0) = 0
    1 1 1 1 1;   % Y3(1) = Yf3
    0 1 2 3 4    % V3(1) = 0
];
B_Y3 = [Yf2; 0; 1; 0]; 

% 对Z3(t)的约束 (4次方)
A_Z3 = [
    1 0 0 0 0;   % Z3(0) = Zf2
    0 1 0 0 0;   % Vz3(0) = 0
    1 1 1 1 1;   % Z3(1) = Zf3
    0 1 2 3 4    % Vz3(1) = 0
];
B_Z3 = [Zf2; 0; -2; 0]; 

% 解多项式系数
coeff_X3 = A_X3 \ B_X3;  
coeff_Y3 = A_Y3 \ B_Y3;  
coeff_Z3 = A_Z3 \ B_Z3;  

% 输出结果
disp('X1(t)的多项式系数:'); disp(coeff_X1);
disp('Y1(t)的多项式系数:'); disp(coeff_Y1);
disp('Z1(t)的多项式系数:'); disp(coeff_Z1);
disp('X2(t)的多项式系数:'); disp(coeff_X2);
disp('Y2(t)的多项式系数:'); disp(coeff_Y2);
disp('Z2(t)的多项式系数:'); disp(coeff_Z2);
disp('X3(t)的多项式系数:'); disp(coeff_X3);
disp('Y3(t)的多项式系数:'); disp(coeff_Y3);
disp('Z3(t)的多项式系数:'); disp(coeff_Z3);

% 生成路径点的计算
t = linspace(0, 1, 100);  % 时间从0到1
X1_t = polyval(flip(coeff_X1'), t);
Y1_t = polyval(flip(coeff_Y1'), t);
Z1_t = polyval(flip(coeff_Z1'), t);
X2_t = coeff_X2(1) + coeff_X2(2)*t;
Y2_t = coeff_Y2(1) + coeff_Y2(2)*t;
Z2_t = coeff_Z2(1) + coeff_Z2(2)*t;
X3_t = polyval(flip(coeff_X3'), t);
Y3_t = polyval(flip(coeff_Y3'), t);
Z3_t = polyval(flip(coeff_Z3'), t);

% 绘制路径
figure;
hold on;
plot3(X1_t, Y1_t, Z1_t, 'LineWidth', 2, 'DisplayName', 'Path 1');
plot3(X2_t, Y2_t, Z2_t, 'LineWidth', 2, 'DisplayName', 'Path 2');
plot3(X3_t, Y3_t, Z3_t, 'LineWidth', 2, 'DisplayName', 'Path 3');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('路径规划轨迹');
grid on;
legend;
hold off;
