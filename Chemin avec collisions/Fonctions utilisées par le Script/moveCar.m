function moveCar(car, waypoints, index, speed, Ts)

[next_position, next_Yaw, ~] = motionRectiligne(car, waypoints(index+1,1:2), speed, Ts);
if norm(car.Position(1:2)-waypoints(index+1,1:2))<0.3
    index=mod(index,length(waypoints)-1)+1;
end
car.Position=next_position;
car.Yaw=next_Yaw;

end