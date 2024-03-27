function [ind,error_y] = find_nearest_point(vehicle_state_now,road_point)
N = length(road_point(:,1));
distance_min = sqrt((vehicle_state_now(1)-road_point(1,1))^2+(vehicle_state_now(2)-road_point(1,2))^2);
ind = 1;
for i=2:1:N
    temp_distance = sqrt((vehicle_state_now(1)-road_point(i,1))^2+(vehicle_state_now(2)-road_point(i,2))^2);
    if ((temp_distance < distance_min) && (abs(vehicle_state_now(3)-road_point(i,3)) <= pi/2 || abs(vehicle_state_now(3)-road_point(i,3)) >= 3*pi/2))
        distance_min = temp_distance;
        ind = i;
    end
end  

% for i=1:1:N
%     distance(i)=sqrt((vehicle_state_now(1)-road_point(i,1))^2+(vehicle_state_now(2)-road_point(i,2))^2);
% end
% [distance_min,ind] = min(distance);
    
angle = atan2(road_point(ind,2)-vehicle_state_now(2),road_point(ind,1)-vehicle_state_now(1));  %最近点与车辆连线的角度
theta_e = mod((angle-vehicle_state_now(3))+2*pi,2*pi); %最近点与车辆连线 与 车辆横摆角 的角度之差
    if theta_e<=pi 
        error_y = distance_min;
    else
        error_y = -distance_min;
    end
end