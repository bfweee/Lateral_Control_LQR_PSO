function road_point=path_matching(x_desired,y_desired)
cx = x_desired;              %离散路径点横坐标
cy = y_desired;              %离散路径点纵坐标
dl = sqrt(diff(cx).^2+diff(cy).^2);   %一阶差分计算
l=cumsum(dl,2);             %每一行依次累加求和
l=[0 l];                    %每个路径点对于的行驶路程s
ppx = spline(l, cx);        %针对x=x(s),y=y(s)三次样条插值
ppy = spline(l, cy);

dppx = ppx;
dppx.coefs=ppx.coefs(:,1:end-1).*kron(ones(size(ppx.coefs,1),1),(size(ppx.coefs,2)-1):-1:1);
dppx.order = ppx.order-1;    %注意：易遗漏
dppy = ppy;
dppy.coefs = ppy.coefs(:,1:end-1).*kron(ones(size(ppy.coefs,1),1),(size(ppy.coefs,2)-1):-1:1);
dppy.order = ppy.order-1;

ddppx = dppx;
ddppx.coefs=dppx.coefs(:,1:end-1).*kron(ones(size(dppx.coefs,1),1),(size(dppx.coefs,2)-1):-1:1);
ddppx.order = dppx.order-1;
ddppy = dppy;
ddppy.coefs = dppy.coefs(:,1:end-1).*kron(ones(size(dppy.coefs,1),1),(size(dppy.coefs,2)-1):-1:1);
ddppy.order = dppy.order-1;

%离散路径点计算，x y yaw curvature
ds = 0.05;          %调节离散路径点间距
s = 0:ds:l(end);
x = ppval(ppx, s);
y = ppval(ppy, s);
curvature = ((ppval(dppx,s).*ppval(ddppy,s)-ppval(dppy,s).*ppval(ddppx,s))) ./ (ppval(dppx,s).^2+ppval(dppy,s).^2).^(3/2);  
yaw = mod(atan2(ppval(dppy,s),ppval(dppx,s))+2*pi,2*pi);

road_point=[x',y',yaw',curvature',s'];
end