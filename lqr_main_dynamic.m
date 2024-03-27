% ���ڳ��������ɶȶ���ѧ���ģ�͵�����ϵͳ����ʱ��LQR���ƣ�����V�㶨��


clc;
clear all;

%% �ο�·������,������ɢ·���������Լ�����
[X_xout,Y_xout]=S_turn();
road_point = path_matching(X_xout,Y_xout);
% figure(1);
% plot(road_point(:,1),road_point(:,2),'r','Linewidth',2);
% xlabel('������X/m');
% ylabel('������Y/m');
% legend('����·��');
% hold off;
% figure(2);
% plot(road_point(:,5),road_point(:,4),'r','Linewidth',2);
% xlabel('·��s/m');
% ylabel('����/��1/m��');
% legend('�ο�����');
% hold off;
% figure(3);
% plot(road_point(:,5),road_point(:,3),'r','Linewidth',2);
% xlabel('·��s/m');
% ylabel('�ο���ڽ�/rad');
% legend('�ο���ڽ�');
% hold off;


%% ��������
dt = 0.02;              %��������
Q = [4 0 0 0;           %����仯��Ȩ�ؾ���
     0 1 0 0;
     0 0 1 0;
     0 0 0 1;];
R = 2*eye(1);           %������Ȩ�ؾ���
steer_max = 35/180*pi;  %ǰ��ת�Ƿ�ΧԼ��
vehicle_state_now = [0  0 0 15 0 0];  %������ǰ��ʼ״̬[x,y,yaw,Vx,Vy,yaw_rate];
vehicle_state_now1 = [0  0 0 15 0 0];
vehicle_state_now2 = [0  0 0 15 0 0];
%% ǰ������
%%����a,b,m,cf,cr,k,kr

%% �������ٶȡ�����������AB����Ķ��塿
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

%% ��PSO�������á�
w = 0.6;    % ��������
c1 = 2;     % Ȩ������
c2 = 2;         

Dim = 5;    % ά��
SwarmSize = 20;     % ����Ⱥ��ģ
MaxIter = 50;      % ����������
MinFit = 0.03;       % ��С��Ӧֵ
Vmax = 0.2;   % ����ٶ�
Vmin = -0.2;  % ��С�ٶ�
Ub = [100 100 100 100 100];   % �����Ͻ�
Lb = [1 1 1 1 1]; % �����½�
ObjFun = @PSO_LQR;  % ���Ż��������

xr = road_point(:,1)';
yr = road_point(:,2)';
thetar = road_point(:,3)';
kappar = road_point(:,4)';
%% ����ʼ��PSO��
Range = ones(SwarmSize,1)*(Ub - Lb);
% ��ʼ������Ⱥ���ٶ�
Swarm = rand(SwarmSize, Dim).*Range + ones(SwarmSize,1)*Lb;
VStep = rand(SwarmSize, Dim)*(Vmax - Vmin) + Vmin;
% ��ʼ������Ⱥ����Ӧֵ
fSwarm = zeros(SwarmSize, 1);
kSwarm = zeros(SwarmSize, 4);   % ��¼��ͬ��Ⱥ��Kֵ
for i = 1 : SwarmSize
    [fSwarm(i,:), kSwarm(i,:)] = feval(ObjFun, A, B, Swarm(i,:));
end
% ��С��Ӧֵ��ֵ������
[bestf,bestindex] = min(fSwarm);
zbest = Swarm(bestindex,:);     % ȫ�����
kbest = kSwarm(bestindex,:);    % �����Ӧ�ȶ�Ӧ��K
gbest = Swarm;      % �������
fzbest = bestf;     % ȫ�������Ӧֵ
fgbest = fSwarm;    % ���������Ӧֵ

%% PSO������ѭ���������ŵ�QR
iter = 0;
while((iter < MaxIter) && (fzbest > MinFit))
    for j = 1: SwarmSize
        %%**********************************
        % �ٶȸ���
        VStep(j,:) = w*VStep(j,:) + c1*rand*(gbest(j,:)-Swarm(j,:)) + c2*rand*(zbest - Swarm(j,:));
        if VStep(j,:) > Vmax, VStep(j,:) = Vmax; end
        if VStep(j,:) < Vmin, VStep(j,:) = Vmin; end
        
        % λ�ø���
        Swarm(j,:) = Swarm(j,:) + VStep(j,:);
        for k = 1 : Dim
            if Swarm(j, k) > Ub(k), Swarm(j,k) = Ub(k); end
            if Swarm(j, k) < Lb(k), Swarm(j,k) = Lb(k); end
        end
        %%**********************************
        % �������Ÿ���
        [fSwarm(j,:), kSwarm(j,:)] = feval(ObjFun, A, B, Swarm(j,:));
        if fSwarm(j) < fgbest(j)
            gbest(j,:) = Swarm(j,:);
            fgbest(j) = fSwarm(j);
        end
        
        % Ⱥ�����Ÿ���
        if fSwarm(j) < fzbest
            zbest = Swarm(j,:);
            fzbest = fSwarm(j);
            kbest = kSwarm(j,:);
        end
    end
    iter = iter + 1;     % ������������
end
% �������ռ�����ݱ���
save('results.mat');

%% LQR��������
i = 1;%���� i ������ѭ���ļ�������ѭ���е�Ŀ����ģ�⳵�����Ÿ�����·��ʻ�Ĺ��̡�
vehicle_state_all(i,:) = vehicle_state_now;%�洢����������״̬
ind = 0;
ind_end = length(road_point(:,1));

%% PSO+LQR+FF
while (ind<ind_end)
    [ind,error_y] = find_nearest_point(vehicle_state_now,road_point);
    nearest_point = road_point(ind,:);
    %delta_fb= lqr_control(vehicle_state_now,nearest_point,Q,R,error_y,steer_max);
    delta_fb = pso_lqr_control(vehicle_state_now,nearest_point,kbest,error_y,steer_max);
    kr = kappar(ind);%����
    delta_ff = 0;
    delta_ff = kr*(a+b-b*kbest(3)-(m*v*v/(a+b))*((b/cf)+(a/cr)*kbest(3)-(a/cr)));
    %delta_ff = kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    delta_f = delta_fb + delta_ff;
    vehicle_state_now = vehicle_update(vehicle_state_now,delta_f,dt,steer_max);
    
    %��¼��Ϊ��һ������׼��
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
    legend('�ο��켣','����λ��');
end 
%% LQR+FF
i = 1;%���� i ������ѭ���ļ�������ѭ���е�Ŀ����ģ�⳵�����Ÿ�����·��ʻ�Ĺ��̡�
vehicle_state_all1(i,:) = vehicle_state_now1;%�洢����������״̬
ind1 = 0;
ind_end1 = length(road_point(:,1));
while (ind1<ind_end1)
    [ind1,error_y1] = find_nearest_point(vehicle_state_now1,road_point);
    nearest_point = road_point(ind1,:);
    delta_fb= lqr_control(vehicle_state_now1,nearest_point,Q,R,error_y1,steer_max);
    kr = kappar(ind1);%����
    delta_ff = kr*(a+b-b*kbest(3)-(m*v*v/(a+b))*((b/cf)+(a/cr)*kbest(3)-(a/cr)));
    %delta_ff = kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    delta_f = delta_fb + delta_ff;
    vehicle_state_now1 = vehicle_update(vehicle_state_now1,delta_f,dt,steer_max);
    
    %��¼��Ϊ��һ������׼��
    error_y_all1(i) = error_y1;
    delta_f_all1(i) = delta_f;
    time_all(i) = dt*i;
    vehicle_state_all1(i+1,:) = vehicle_state_now1;
    i = i+1;
end
%% LQR
i = 1;%���� i ������ѭ���ļ�������ѭ���е�Ŀ����ģ�⳵�����Ÿ�����·��ʻ�Ĺ��̡�
vehicle_state_all2(i,:) = vehicle_state_now2;%�洢����������״̬
ind2 = 0;
ind_end2 = length(road_point(:,1));
while (ind2<ind_end2)
    [ind2,error_y2] = find_nearest_point(vehicle_state_now2,road_point);
    nearest_point = road_point(ind2,:);
    delta_fb= lqr_control(vehicle_state_now2,nearest_point,Q,R,error_y2,steer_max);
    kr = kappar(ind2);%����
    delta_ff = 0;
    %delta_ff = kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    delta_f = delta_fb + delta_ff;
    vehicle_state_now2 = vehicle_update(vehicle_state_now2,delta_f,dt,steer_max);
    
    %��¼��Ϊ��һ������׼��
    error_y_all2(i) = error_y2;
    delta_f_all2(i) = delta_f;
    time_all(i) = dt*i;
    vehicle_state_all2(i+1,:) = vehicle_state_now2;
    i = i+1;
end
% ��ͼ
 figure(2);
 plot(road_point(:,1),road_point(:,2),'k','Linewidth',1);
 hold on;
 plot(vehicle_state_all(:,1),vehicle_state_all(:,2),'b','Linewidth',1);
   plot(vehicle_state_all1(:,1),vehicle_state_all2(:,2),'r','Linewidth',1);
    plot(vehicle_state_all2(:,1),vehicle_state_all2(:,2),'g','Linewidth',1);
 xlabel('������X/m');
 ylabel('������Y/m');
 legend('�ο��켣','PSO+LQR+FF','LQR+FF','LQR');
 title('���ٽ��');
 hold off;

 figure(3);
 plot(time_all,delta_f_all,'b','Linewidth',1);
 hold on;
 plot(time_all,delta_f_all1,'r','Linewidth',1);
 plot(time_all,delta_f_all2,'g','Linewidth',1);
 xlabel('ʱ��/t');
 ylabel('ǰ��ת��/rad');
 legend('PSO+LQR+FF','LQR+FF','LQR');
 title('��������')
 hold off;

 figure(4);
 plot(time_all,error_y_all,'b','Linewidth',1);
 hold on; 
 plot(time_all,error_y_all1,'r','Linewidth',1);
 plot(time_all,error_y_all2,'g','Linewidth',1);
 xlabel('ʱ��/t');
 ylabel('�������/m');
 legend('PSO+LQR+FF','LQR+FF','LQR');
 title('�������');
 hold off;
 
