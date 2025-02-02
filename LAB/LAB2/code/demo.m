function [sys,x0,str,ts,simStateCompliance] = demo(t,x,u,flag)
%主函数
%主函数包含四个输出：
%                 sys数组包含某个子函数返回的值
%                 x0为所有状态的初始化向量
%                 str是保留参数，总是一个空矩阵
%                 Ts返回系统采样时间
%函数的四个输入分别为采样时间t、状态x、输入u和仿真流程控制标志变量flag
%输入参数后面还可以接续一系列的附带参数simStateCompliance
%注意：编写自己的S-function时，应该把函数名sfuntmpl改为S-function中对应的名字
switch flag,
  case 0,
      [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
%主函数结束
%% %下面是各个子函数，即各个回调过程
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
%初始化回调子函数
%提供状态、输入、输出、采样时间数目和初始状态的值
%初始化阶段，标志变量flag首先被置为0，S-function首次被调用时
%该子函数首先被调用，且为S-function模块提供下面信息
%该子函数必须存在
sizes = simsizes;             %生成sizes数据结构，信息被包含在其中
sizes.NumContStates  = 4;     %连续状态数，缺省为0
sizes.NumDiscStates  = 0;     %离散状态数，缺省为0
sizes.NumOutputs     = 2;     %输出个数，缺省为0
sizes.NumInputs      = 1;     %输入个数，缺省为0
sizes.DirFeedthrough = 0;     %是否存在直馈通道，1表示存在，0表示不存在
sizes.NumSampleTimes = 1;     %采样时间个数，至少是一个
sys = simsizes(sizes);        %返回size数据结构所包含的信息
x0  = [0 0 pi/4 0];           %设置初始状态
str = [];                     %保留变量置空
ts  = [0 0];                  %设置采样时间
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
    %计算导数回调子函数
    %给定t,x,u计算连续状态的导数，可以在此给出系统的连续状态方程
    %该子函数可以不存在
    M = 1;  % 小车质量
    m = 0.5; % 摆杆质量
    l = 0.5; % 摆杆长度
    g = 9.8; % 重力加速度
    
    % x(1)：位移 ; x(2)：小车速度;  x(3)：偏转角theta; x(4)：角速度;
    
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4);

    % 计算加速度和角加速度
    ddx = (u - m^2*l^2*x4^2*sin(x3) - m*g*sin(x3)*cos(x3)) / (M + m*sin(x3)^2);
    ddtheta = ( m*g*l*sin(x3) -  m*l*cos(x3)*ddx) / (m*l^2);
                
    % 状态导数
    dx1 = x2;
    dx2 = ddx;
    dx3 = x4;
    dx4 = ddtheta;
    
    sys = [dx1; dx2; dx3; dx4];
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
sys = [x(1);x(3)];                     %sys表示输出，即y
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
