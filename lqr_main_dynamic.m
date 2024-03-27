% 基于车辆二自由度动力学误差模型的连续系统无限时间LQR控制（假设V恒定）


clc;
clear all;

%% 参考路径生成,包含离散路径点坐标以及曲率
[X_xout,Y_xout]=S_turn();
road_point = path_matching(X_xout,Y_xout);
% figure(1);
% plot(road_point(:,1),road_point(:,2),'r','Linewidth',2);
% xlabel('横坐标X/m');
% ylabel('纵坐标Y/m');
% legend('期望路径');
% hold off;
% figure(2);
% plot(road_point(:,5),road_point(:,4),'r','Linewidth',2);
% xlabel('路程s/m');
% ylabel('曲率/（1/m）');
% legend('参考曲率');
% hold off;
% figure(3);
% plot(road_point(:,5),road_point(:,3),'r','Linewidth',2);
% xlabel('路程s/m');
% ylabel('参考横摆角/rad');
% legend('参考横摆角');
% hold off;


%% 参数设置
dt = 0.02;              %采样周期
Q = [4 0 0 0;           %误差及其变化率权重矩阵
     0 1 0 0;
     0 0 1 0;
     0 0 0 1;];
R = 2*eye(1);           %控制量权重矩阵
steer_max = 35/180*pi;  %前轮转角范围约束
vehicle_state_now = [0  0 0 15 0 0];  %车辆当前初始状态[x,y,yaw,Vx,Vy,yaw_rate];
vehicle_state_now1 = [0  0 0 15 0 0];
vehicle_state_now2 = [0  0 0 15 0 0];
%% 前馈控制
%%参数a,b,m,cf,cr,k,kr

%% 【纵向速度、车辆参数及AB矩阵的定义】
cf=-110000;
cr=cf;
m=1412;
Iz=1536.7;
a=1.015;
b=2.910-1.015;
v=vehicle_state_now(4);
 A=[0,1,0,0;
        0,(cf+cr)/(m*v),-(cf+cr)/m,(a*cf-b*cr)/(m*v);
        0,0,0,1;
        0,(a*cf-b*cr)/(Iz*v),-(a*cf-b*cr)/Iz,(a*a*cf+b*b*cr)/(Iz*v)];
    B=[0;
        -cf/m;
        0;
        -a*cf/Iz];

%% 【PSO参数设置】
w = 0.6;    % 惯性因子
c1 = 2;     % 权重因子
c2 = 2;         

Dim = 5;    % 维数
SwarmSize = 20;     % 粒子群规模
MaxIter = 50;      % 最大迭代次数
MinFit = 0.03;       % 最小适应值
Vmax = 0.2;   % 最大速度
Vmin = -0.2;  % 最小速度
Ub = [100 100 100 100 100];   % 函数上界
Lb = [1 1 1 1 1]; % 函数下界
ObjFun = @PSO_LQR;  % 待优化函数句柄

xr = road_point(:,1)';
yr = road_point(:,2)';
thetar = road_point(:,3)';
kappar = road_point(:,4)';
%% 【初始化PSO】
Range = ones(SwarmSize,1)*(Ub - Lb);
% 初始化粒子群和速度
Swarm = rand(SwarmSize, Dim).*Range + ones(SwarmSize,1)*Lb;
VStep = rand(SwarmSize, Dim)*(Vmax - Vmin) + Vmin;
% 初始化粒子群的适应值
fSwarm = zeros(SwarmSize, 1);
kSwarm = zeros(SwarmSize, 4);   % 记录不同种群的K值
for i = 1 : SwarmSize
    [fSwarm(i,:), kSwarm(i,:)] = feval(ObjFun, A, B, Swarm(i,:));
end
% 最小适应值的值及索引
[bestf,bestindex] = min(fSwarm);
zbest = Swarm(bestindex,:);     % 全局最佳
kbest = kSwarm(bestindex,:);    % 最佳适应度对应的K
gbest = Swarm;      % 个体最佳
fzbest = bestf;     % 全局最佳适应值
fgbest = fSwarm;    % 个体最佳适应值

%% PSO迭代，循环搜索最优的QR
iter = 0;
while((iter < MaxIter) && (fzbest > MinFit))
    for j = 1: SwarmSize
        %%**********************************
        % 速度更新
        VStep(j,:) = w*VStep(j,:) + c1*rand*(gbest(j,:)-Swarm(j,:)) + c2*rand*(zbest - Swarm(j,:));
        if VStep(j,:) > Vmax, VStep(j,:) = Vmax; end
        if VStep(j,:) < Vmin, VStep(j,:) = Vmin; end
        
        % 位置更新
        Swarm(j,:) = Swarm(j,:) + VStep(j,:);
        for k = 1 : Dim
            if Swarm(j, k) > Ub(k), Swarm(j,k) = Ub(k); end
            if Swarm(j, k) < Lb(k), Swarm(j,k) = Lb(k); end
        end
        %%**********************************
        % 个体最优更新
        [fSwarm(j,:), kSwarm(j,:)] = feval(ObjFun, A, B, Swarm(j,:));
        if fSwarm(j) < fgbest(j)
            gbest(j,:) = Swarm(j,:);
            fgbest(j) = fSwarm(j);
        end
        
        % 群体最优更新
        if fSwarm(j) < fzbest
            zbest = Swarm(j,:);
            fzbest = fSwarm(j);
            kbest = kSwarm(j,:);
        end
    end
    iter = iter + 1;     % 迭代次数更新
end
% 将工作空间的内容保存
save('results.mat');

%% LQR控制主体
i = 1;%变量 i 被用作循环的计数器。循环中的目的是模拟车辆沿着给定道路行驶的过程。
vehicle_state_all(i,:) = vehicle_state_now;%存储车辆的所有状态
ind = 0;
ind_end = length(road_point(:,1));

%% PSO+LQR+FF
while (ind<ind_end)
    [ind,error_y] = find_nearest_point(vehicle_state_now,road_point);
    nearest_point = road_point(ind,:);
    %delta_fb= lqr_control(vehicle_state_now,nearest_point,Q,R,error_y,steer_max);
    delta_fb = pso_lqr_control(vehicle_state_now,nearest_point,kbest,error_y,steer_max);
    kr = kappar(ind);%曲率
    delta_ff = 0;
    delta_ff = kr*(a+b-b*kbest(3)-(m*v*v/(a+b))*((b/cf)+(a/cr)*kbest(3)-(a/cr)));
    %delta_ff = kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    delta_f = delta_fb + delta_ff;
    vehicle_state_now = vehicle_update(vehicle_state_now,delta_f,dt,steer_max);
    
    %记录，为下一个点做准备
    error_y_all(i) = error_y;
    delta_f_all(i) = delta_f;
    time_all(i) = dt*i;
    vehicle_state_all(i+1,:) = vehicle_state_now;
    i = i+1;
    
    figure(1);
    plot(road_point(:,1),road_point(:,2),'k','Linewidth',1);
    hold on;
    plot(vehicle_state_all(i-1,1),vehicle_state_all(i-1,2),'r*');
    hold on;
    legend('参考轨迹','车辆位置');
end 
%% LQR+FF
i = 1;%变量 i 被用作循环的计数器。循环中的目的是模拟车辆沿着给定道路行驶的过程。
vehicle_state_all1(i,:) = vehicle_state_now1;%存储车辆的所有状态
ind1 = 0;
ind_end1 = length(road_point(:,1));
while (ind1<ind_end1)
    [ind1,error_y1] = find_nearest_point(vehicle_state_now1,road_point);
    nearest_point = road_point(ind1,:);
    delta_fb= lqr_control(vehicle_state_now1,nearest_point,Q,R,error_y1,steer_max);
    kr = kappar(ind1);%曲率
    delta_ff = kr*(a+b-b*kbest(3)-(m*v*v/(a+b))*((b/cf)+(a/cr)*kbest(3)-(a/cr)));
    %delta_ff = kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    delta_f = delta_fb + delta_ff;
    vehicle_state_now1 = vehicle_update(vehicle_state_now1,delta_f,dt,steer_max);
    
    %记录，为下一个点做准备
    error_y_all1(i) = error_y1;
    delta_f_all1(i) = delta_f;
    time_all(i) = dt*i;
    vehicle_state_all1(i+1,:) = vehicle_state_now1;
    i = i+1;
end
%% LQR
i = 1;%变量 i 被用作循环的计数器。循环中的目的是模拟车辆沿着给定道路行驶的过程。
vehicle_state_all2(i,:) = vehicle_state_now2;%存储车辆的所有状态
ind2 = 0;
ind_end2 = length(road_point(:,1));
while (ind2<ind_end2)
    [ind2,error_y2] = find_nearest_point(vehicle_state_now2,road_point);
    nearest_point = road_point(ind2,:);
    delta_fb= lqr_control(vehicle_state_now2,nearest_point,Q,R,error_y2,steer_max);
    kr = kappar(ind2);%曲率
    delta_ff = 0;
    %delta_ff = kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    delta_f = delta_fb + delta_ff;
    vehicle_state_now2 = vehicle_update(vehicle_state_now2,delta_f,dt,steer_max);
    
    %记录，为下一个点做准备
    error_y_all2(i) = error_y2;
    delta_f_all2(i) = delta_f;
    time_all(i) = dt*i;
    vehicle_state_all2(i+1,:) = vehicle_state_now2;
    i = i+1;
end
% 画图
 figure(2);
 plot(road_point(:,1),road_point(:,2),'k','Linewidth',1);
 hold on;
 plot(vehicle_state_all(:,1),vehicle_state_all(:,2),'b','Linewidth',1);
   plot(vehicle_state_all1(:,1),vehicle_state_all2(:,2),'r','Linewidth',1);
    plot(vehicle_state_all2(:,1),vehicle_state_all2(:,2),'g','Linewidth',1);
 xlabel('横坐标X/m');
 ylabel('纵坐标Y/m');
 legend('参考轨迹','PSO+LQR+FF','LQR+FF','LQR');
 title('跟踪结果');
 hold off;

 figure(3);
 plot(time_all,delta_f_all,'b','Linewidth',1);
 hold on;
 plot(time_all,delta_f_all1,'r','Linewidth',1);
 plot(time_all,delta_f_all2,'g','Linewidth',1);
 xlabel('时间/t');
 ylabel('前轮转角/rad');
 legend('PSO+LQR+FF','LQR+FF','LQR');
 title('航向角误差')
 hold off;

 figure(4);
 plot(time_all,error_y_all,'b','Linewidth',1);
 hold on; 
 plot(time_all,error_y_all1,'r','Linewidth',1);
 plot(time_all,error_y_all2,'g','Linewidth',1);
 xlabel('时间/t');
 ylabel('横向误差/m');
 legend('PSO+LQR+FF','LQR+FF','LQR');
 title('跟踪误差');
 hold off;
 
