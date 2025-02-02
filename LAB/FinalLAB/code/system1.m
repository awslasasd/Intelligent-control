function [sys,x0,str,ts,simStateCompliance] = system1(t,x,u,flag)
%主函数
%主函数包含四个输出：
%                 sys数组包含某个子函数返回的值
%                 x0为所有状态的初始化向量
%                 str是保留参数，总是一个空矩阵
%                 Ts返回系统采样时间
%函数的四个输入分别为采样时间t、状态x、输入u和仿真流程控制标志变量flag
%输入参数后面还可以接续一系列的附带参数simStateCompliance
% 根据flag的值调用对应的回调函数
switch flag
    case 0
        % 初始化
        [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes;
    case 1
        % 计算连续状态导数
        sys = mdlDerivatives(t,x,u);
    case 2
        % 更新离散状态（如果有）
        sys = mdlUpdate(t,x,u);
    case 3
        % 计算输出
        sys = mdlOutputs(t,x,u);
    case 4
        % 下一个采样时间（用于可变步长仿真）
        sys = mdlGetTimeOfNextVarHit(t,x,u);
    case 9
        % 仿真结束前的清理工作
        sys = mdlTerminate(t,x,u);
    otherwise
        % 未处理的flag错误
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
% 主函数结束

%% %下面是各个子函数，即各个回调过程
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
%初始化回调子函数
%提供状态、输入、输出、采样时间数目和初始状态的值
%初始化阶段，标志变量flag首先被置为0，S-function首次被调用时
%该子函数首先被调用，且为S-function模块提供下面信息
%该子函数必须存在
sizes = simsizes;             %生成sizes数据结构，信息被包含在其中
sizes.NumContStates  = 6;     %连续状态数，缺省为0
sizes.NumDiscStates  = 0;     %离散状态数，缺省为0
sizes.NumOutputs     = 1;     %输出个数，缺省为0
sizes.NumInputs      = 1;     %输入个数，缺省为0
sizes.DirFeedthrough = 0;     %是否存在直馈通道，1表示存在，0表示不存在
sizes.NumSampleTimes = 1;     %采样时间个数，至少是一个
sys = simsizes(sizes);        %返回size数据结构所包含的信息
x0  = [0 0 0 0 0 0];              %设置初始状态
str = [];                     %保留变量置空
ts  = [0 0];                  %设置采样时间
simStateCompliance = 'UnknownSimState';

% 计算导数回调子函数
function sys = mdlDerivatives(t, x, u)
    % 计算连续状态导数
    
    % 状态变量
    r       = x(1);
    theta1  = x(2);
    theta2  = x(3);
    r_dot   = x(4);
    theta1_dot = x(5);
    theta2_dot = x(6);
    
    % 输入变量
    u_input = u;    % 控制电压 u
    
    % 系统参数
    M0 = 1.6;            % 小车系统的等效质量 (kg)
    M1 = 0.185;          % 下摆的质量 (kg)
    M2 = 0.2;            % 上摆的质量 (kg)
    F0 = 21.8519;        % 系统的等效摩擦阻力系数 (N*m/s)
    F1 = 0.006415;       % 下摆的摩擦阻力系数 (N*m/s)
    F2 = 0.006717;       % 上摆的摩擦阻力系数 (N*m/s)
    L  = 0.483;          % 下摆轴心到上摆轴心距离 (m)
    l1 = 0.283;          % 下摆重心到其轴心距离 (m)
    l2 = 0.245;          % 上摆重心到其轴心距离 (m)
    G0 = 7.6889;         % 控制力与控制电压之比 (N/V)
    J1 = 0.00547;        % 下摆对其重心的转动惯量 (kg*m^2)
    J2 = 0.00549;        % 上摆对其重心的转动惯量 (kg*m^2)
    g  = 9.81;           % 重力加速度 (m/s^2)
    
    % 构建 M 矩阵
    M_matrix = [ ...
        M0 + M1 + M2, (M1*l1 + M2*L)*cos(theta1), M2*l2*cos(theta2);
        (M1*l1 + M2*L)*cos(theta1), J1 + M1*l1^2 + M2*L^2, M2*L*l2*cos(theta2 - theta1);
        M2*l2*cos(theta2), M2*L*l2*cos(theta2 - theta1), J2 + M2*l2^2 ...
        ];
    
    % 构建 F 矩阵
    F_matrix = [ ...
        F0, - (M1*l1 + M2*L)*sin(theta1)*theta1_dot, - M2*l2*sin(theta2)*theta2_dot;
        0, F1 + F2, -F2 - M2*L*l2*sin(theta2 - theta1)*theta2_dot;
        0, M2*L*l2*sin(theta2 - theta1)*theta1_dot - F2, F2 ...
        ];
    
    % 构建 N 向量
    N_vector = [ ...
        0;
        (M1*l1 + M2*L)*g*sin(theta1);
        M2*L*g*sin(theta2) ...
        ];
    
    % 输入向量
    input_vector = [G0; 0; 0] * u_input;
    
    % 计算加速度
    acceleration = M_matrix \ ( -F_matrix * [r_dot; theta1_dot; theta2_dot] + N_vector + input_vector );
    
    % 提取加速度
    r_ddot      = acceleration(1);
    theta1_ddot = acceleration(2);
    theta2_ddot = acceleration(3);
    
    % 状态导数
    dxdt = [ ...
        r_dot;          % dr/dt
        theta1_dot;     % dtheta1/dt
        theta2_dot;     % dtheta2/dt
        r_ddot;         % d(r_dot)/dt
        theta1_ddot;    % d(theta1_dot)/dt
        theta2_ddot     % d(theta2_dot)/dt
        ];
    
    sys = dxdt;

%% 
function sys=mdlUpdate(t,x,u)
%状态更新回调子函数
%给定t、x、u计算离散状态的更新
%每个仿真步内必然调用该子函数，不论是否有意义
%除了在此描述系统的离散状态方程外，还可以在此添加其他每个仿真步内都必须执%行的代码
sys = [];                     %sys表示下一个离散状态，即x(k+1)
%% 
function sys=mdlOutputs(t,x,u)
%计算输出回调函数
%给定t,x,u计算输出，可以在此描述系统的输出方程
%该子函数必须存在
sys = [x(1)];                     %sys表示输出，即y
%% 
function sys=mdlGetTimeOfNextVarHit(t,x,u)
%计算下一个采样时间
%仅在系统是变采样时间系统时调用
sampleTime = 1;               %设置下一次采样时间是在1s以后
sys = t + sampleTime;         %sys表示下一个采样时间点
%% 
function sys=mdlTerminate(t,x,u)
%仿真结束时要调用的回调函数
%在仿真结束时，可以在此完成仿真结束所需的必要工作
sys = [];
