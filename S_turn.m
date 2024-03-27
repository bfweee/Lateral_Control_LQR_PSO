function [xc,yc]=S_turn()
%% S路径
% xc = 0:0.2:400;                          %给定路径的横坐标cx取值，共501个点
% for i = 1:length(cx)                   
%     yc(i) = sin(cx(i)/40)*cx(i)/12;       %通过循环，对每个cx对应的cy通过函数关系赋值，得到给定路径的501个路点
% end

% 双移线
shape = 2.4;            
dx1 = 25;
dx2 = 21.95;      
dy1 = 4.05;
dy2 = 5.7;    
Xs1 = 27.19; 
Xs2 = 56.46;
xc = 0:0.1:100;
Nr = length(xc);
for i=1:1:Nr
  z1 = shape/dx1*(xc(i)-Xs1)-shape/2;
  z2 = shape/dx2*(xc(i)-Xs2)-shape/2;
  yc(i) = dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
end

% 大圆弧路径
% R = 150;
% x_center = 0;
% y_center = 150;
% theta = -pi/2:0.002:pi/2;
% for i=1:1:length(theta)
%     xc(i) = x_center+R*cos(theta(i));
%     yc(i) = y_center+R*sin(theta(i));
% end

end