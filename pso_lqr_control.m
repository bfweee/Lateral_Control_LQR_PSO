function delta_f = pso_lqr_control(vehicle_state_now,nearest_point,k,error_y,steer_max)
%% 车辆参数及状态方程A、B矩阵
% lf = 1.232;                                       %前后车轮距离车辆质心的距离，车辆固有参数
% lr = 1.468;
% Ccf = 66900; Ccr = 62700; Clf = 66900; Clr = 62700;     %纵横向侧偏刚度
% m = 1723; g = 9.8; I = 4175;                            %质量、重力加速度、转动惯量
% 
% v=vehicle_state_now(4);
% A = [0 1 0 0;
%      0 -2*(Ccf+Ccr)/m/v 2*(Ccf+Ccr)/m  (-2*Ccf*lf+2*Ccr*lr)/m/v;
%      0 0 0 1;
%      0 (-2*Ccf*lf+2*Ccr*lr)/I/v  (2*Ccf*lf-2*Ccr*lr)/I  (-2*Ccf*lf^2-2*Ccr*lr^2)/I/v;];
% B = [0 ; 2*Ccf/m ; 0 ; 2*Ccf*lf/I;];
cf=-110000;%这两个参数是完全一样的
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
 B=[0;-cf/m;0;-a*cf/Iz];

error_yaw = mod(vehicle_state_now(3) - nearest_point(3)+pi,2*pi)-pi;
derror_y = vehicle_state_now(5)*cos(error_yaw)+vehicle_state_now(4)*sin(error_yaw);
derror_yaw = vehicle_state_now(6)-vehicle_state_now(4)*nearest_point(4);
X_feedback = [-error_y ; derror_y ; error_yaw ; derror_yaw];
%% [lqr]
% [K,~,~] = lqr(A,B,Q,R);
% %delta_f = -K*X_feedback-X_feedback(1)*30;
% delta_f = -K*X_feedback;
% delta_f = max(min(steer_max,delta_f),-steer_max);  %前轮转角范围约束
%% PSO_lqr
kbest = k;
delta_f = -kbest*X_feedback;
delta_f = max(min(steer_max,delta_f),-steer_max); 
end