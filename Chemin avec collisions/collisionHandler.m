if isempty(objectDetections) %Pas d'objets détectés
    if reached == 1
        if avoiding_collision == 1
            avoiding_collision = 0;
            road_graph = rmedge(road_graph, waypoints(current_node),waypoints(current_node + 1*direction));
            if direction == 1 %Aller
                [new_waypoints,~] = shortestpath(road_graph, waypoints(current_node), getNodeId(noeud_livraison(1), noeud_livraison(2), TAILLE_CARRE));
                waypoints = [waypoints(1:current_node-1), new_waypoints];
            else %retour
                [new_waypoints,~] = shortestpath(road_graph, getNodeId(0,0, TAILLE_CARRE), waypoints(current_node));
                waypoints = [new_waypoints, waypoints(current_node-1,end)];
            end
        end
        %verification si point de livraison atteint ; la voiture retourne au
        %point de depart
        if norm(egoCar.Position-coord_livraison)<1e-3 | current_node == 0
            if Livraison_OK==0
                Livraison_OK=1;
                point_livraison.PlotColor = 'g';
                disp('livré');
                coord_livraison = [0 0 0];
                direction = -1;
            else
                disp('arrivé');
                fin=1;
            end
        end
        current_node = current_node + 1*direction;
    end
    [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(1), Ts);
    
    
else %Objets détectés dans le périmètre
    global_timout=0;
    for object=1:length(objectDetections) %loop with the number of detected objects
        if flag_coli(j,object,iteration-1)==1 %checking if collision
            
            %[next_position, next_Yaw, reached, timeout, avoiding_collision] = collisionHandler(iteration,...
            %   flag_coli, j, object, current_node, direction, next_position, next_Yaw, reached,...
            %  vec_control, waypoints, TAILLE_CARRE, L, CAR_SPEED, Ts, timeout);
            
            if iteration<2 || flag_coli(j,object,iteration-2)==0 %If initial collision imminent then stop
                
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
end