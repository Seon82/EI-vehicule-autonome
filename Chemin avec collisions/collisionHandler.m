if flag=1 %Collision détectée dans le périmètre
            
            if iteration<2 || flag_mat(j,object,iteration-2)==0 %If initial collision imminent then stop
                
                wait=0;
                
            elseif wait==10 %If collision but timeout expired then go back
                current_node = current_node - 1*direction;
                [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(1), Ts);
                avoiding_collision=1;
                wait=0;
                
            else %If timeout not expired then stop
                wait=wait+1;
                wait;
                
            end
            
            
        else %Object detected but no collision
            if j==1
                [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(1), Ts);
                if reached == 1
                    current_node = current_node + 1*direction;
                end
                
                wait=0;
            end
        end
end