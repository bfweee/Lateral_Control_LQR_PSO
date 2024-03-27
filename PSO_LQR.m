%% PSO_LQR函数，通过不同的QR求解出K，跑Simulink模型确定适应度函数
function [f,k] = PSO_LQR(A, B, qr)
    Q = [qr(1),0,0,0;0,qr(2),0,0;0,0,qr(3),0;0,0,0,qr(4)];
    R = qr(5);
    a = 0.505;
    % 进行LQR求解
    K = lqr(A, B, Q, R);
    
    % 第一列作为时间戳，防止报错
    k1 = [0, K(1)];
    k2 = [0, K(2)];
    k3 = [0, K(3)];
    k4 = [0, K(4)];    
    assignin('base','k1',k1);
    assignin('base','k2',k2);
    assignin('base','k3',k3);
    assignin('base','k4',k4);
    
    % 运行模型并将结果输出到工作空间
    simOut = sim('pso_lqr_test');
    assignin('base','simOut',simOut);
    
    % 计算适应度函数
    %err = simOut.err;
    err = simOut.err.Data;
    assignin('base','err',err);
    
    [m,n] = size(err);
    fitness = a*sum(err(:,1).^2) / m + (1-a)*sum(err(:,3).^2) / m;

    f = fitness;
    k = K;
end

