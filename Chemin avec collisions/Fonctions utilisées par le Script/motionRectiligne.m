function [next_Position, next_Yaw, reached] = motionRectiligne(car,destination,v,Ts)
    tol_dist = 1e-1;
    tol_angle = 1;
    reached = 0;
    next_Position=car.Position;
    if norm(car.Position(1:2)- destination)>tol_dist
            next_Yaw = -rad2deg(atan( (car.Position(2)-destination(2))/(car.Position(1)-destination(1)) ));
            if abs(next_Yaw)<tol_angle && car.Position(1)> destination(1)
                next_Yaw = 180;
            end
            next_Position(1) = car.Position(1) + cos(deg2rad(next_Yaw))*v*Ts;
            next_Position(2) = car.Position(2) + sin(deg2rad(next_Yaw))*v*Ts;
            if sign(car.Position(1)-destination(1))~=sign(next_Position(1)-destination(1))
                car.Position(1) = destination(1);
                y_ok = 1;
            end
            if sign(car.Position(2)-destination(2))~=sign(next_Position(2)-destination(2))
                car.Position(2) = destination(2);
                x_ok = 1;
            end
    else
            reached = 1;
            next_Yaw=car.Yaw;
    end
end