function vehicle_state_update = vehicle_update(vehicle_state_now,delta_f,dt,steer_max)
lf = 1.232;                                       %前后车轮距离车辆质心的距离，车辆固有参数
lr = 1.468;
L = 2.8;
Ccf = 66900; Ccr = 62700; Clf = 66900; Clr = 62700;     %纵横向侧偏刚度
m = 1723; g = 9.8; I = 4175;                            %质量、重力加速度、转动惯量

x = vehicle_state_now(1);
y = vehicle_state_now(2);
yaw = vehicle_state_now(3);
Vx = vehicle_state_now(4);
Vy = vehicle_state_now(5);
yaw_rate = vehicle_state_now(6);

dVy = (-m*Vx*yaw_rate + 2*(Ccf*(delta_f-(Vy+lf*yaw_rate)/Vx)+Ccr*(lr*yaw_rate-Vy)/Vx))/m;
dyaw_rate=(2*(lf*Ccf*(delta_f-(Vy+lf*yaw_rate)/Vx)-lr*Ccr*(lr*yaw_rate-Vy)/Vx))/I;


x = x + (Vx*cos(yaw)-Vy*sin(yaw))*dt;
y = y + (Vx*sin(yaw)+Vy*cos(yaw))*dt;
yaw = yaw + yaw_rate*dt;
Vx = Vx;
Vy = Vy + dVy*dt;
yaw_rate = yaw_rate + dyaw_rate*dt;

vehicle_state_update = [x,y,yaw,Vx,Vy,yaw_rate];

end