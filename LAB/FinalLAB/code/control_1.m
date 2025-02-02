function [sys,x0,str,ts,simStateCompliance] = control_1(t,x,u,flag)
% control_1.m
% -------------------------------------------------------------------------
% 输入:  u(1) = r_d  (参考位置)
%        u(2) = r    (实际位置)
%
% 输出:  sys = 控制输入 u(t)
%
% 连续状态: x(1..5) = W(1..5) (RBF 神经网络权重)
% -------------------------------------------------------------------------

switch flag
    case 0
        [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes();
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    otherwise
        sys = [];
end

end % end of main function


%%=========================================================================
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()
    % 初始化
    sizes = simsizes;
    sizes.NumContStates  = 5;  % 5 个连续状态 -> 神经网络权重
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 1;  % 输出只有一个控制输入
    sizes.NumInputs      = 2;  % 输入: [r_d, r]
    sizes.DirFeedthrough = 1;  % 有直通
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);

    % 初始权重 (可自行修改)
    x0  = 0.1*ones(1,5);

    str = [];
    ts  = [0 0];  % 连续系统
    simStateCompliance = 'UnknownSimState';

    % 全局参数
    global b c lambda gamma eta
    b      = 0.1;     % RBF宽度 
    lambda = 13;      % 误差反馈增益 
    gamma  = 500;     % 自适应增益
    eta    = 3;       % 滑模开关项增益
    
    % RBF 网络中心位置向量
    c = [-2, -1, 0, 1, 2];  % 1 x 5
end


%%=========================================================================
function sys = mdlDerivatives(t,x,u)

    global b c lambda gamma

    % 1. 误差 e = r - r_d
    r_d = u(1);
    r   = u(2);
    e   = r - r_d;

    % 2. 滑模面 (一阶简化) s = e
    s = e;

    % 3. RBF 特征向量 h(r)
    W  = x(1:5);
    h  = zeros(5,1);
    for j = 1:5
        % c(j) 为该中心, 尺寸 1 x 1
        h(j) = exp( - (r - c(j))^2 / (2*b^2) );
    end

    % 4. 自适应律
    dW = gamma * s * h;

    sys = dW;  % 返回列向量

end


%%=========================================================================
function sys = mdlOutputs(t,x,u)
    % 计算控制律

    global b c lambda eta
    
    % 提取输入信号
    r_d = u(1);
    r   = u(2);

    % 1. 误差 e = r - r_d
    e = r - r_d;

    % 2. RBF 输出
    W = x(1:5);  % 当前的神经网络权重
    h = zeros(5,1);
    for j = 1:5
        h(j) = exp( - (r - c(j))^2 / (2*b^2) );
    end
    fn = W'*h;  % 神经网络对未知系统的估计

    % 3. 控制律 (简化一阶滑模)
    ut = -lambda*e - fn - eta*sign(e);

    sys = ut;  % 输出控制量

end
