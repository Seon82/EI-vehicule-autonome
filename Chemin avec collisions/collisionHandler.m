function [next_position, next_Yaw, reached, timeout, avoiding_collision] = collisionHandler(iteration,...
    flag_coli, j, object, current_node, direction,next_position, next_Yaw, reached, vec_control, waypoints, TAILLE_CARRE, L, CAR_SPEED, Ts, timeout)

if iteration<2 || flag_coli(j,object,iteration-2)==0 %If initial collision imminent then stop
    
    timeout=0;
    avoiding_collision=NaN;

elseif timeout==10 %If collision but timeout expired then go back
    current_node = current_node - 1*direction;
    [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(j), Ts);
    avoiding_collision=1;
    timeout=0;
    
else %If timeout not expired then stop
    timeout=timeout+1;
    avoiding_collision=NaN;
    
end

end