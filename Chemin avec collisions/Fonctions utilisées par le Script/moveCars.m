function [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts)

waypoint=waypoints{1,j};
[next_Position, next_Yaw, reached] = motionRectiligne(cars(j),noeuds(waypoint(wp_index(j))),CAR_SPEED(j),Ts);
%Include collisionHandler somewhere here
cars(j).Yaw = next_Yaw;
cars(j).Position = next_Position;
if reached==1
    wp_index(j) = wp_index(j)+1;
end
        
end