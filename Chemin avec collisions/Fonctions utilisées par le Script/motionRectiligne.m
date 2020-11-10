function [next_Position, next_Yaw, reached] = motionRectiligne(car,destination,v,Ts)
    reached = 0;
    next_Position=car.Position;
    distance = norm(car.Position - destination);
    if distance>v*Ts
        next_Yaw = atan2d( destination(2) - car.Position(2), destination(1) - car.Position(1));
        next_Position(1) = car.Position(1) + cos(deg2rad(next_Yaw))*v*Ts;
        next_Position(2) = car.Position(2) + sin(deg2rad(next_Yaw))*v*Ts;
    else
            reached = 1;
            next_Position = destination;
            next_Yaw=car.Yaw;
    end
end