function [timeout, graphe, carStopped, previousFlags, wp_index, waypoints, timeout_threshold] = collisionHandler(flag, j, previousFlags, carStopped, timeout, timeout_threshold, wp_index, waypoints, graphe)

if flag > 0
    if previousFlags(j)==0 %If initial collision imminent then stop
        timeout(j)=0;
        carStopped(j)=1;
        timeout_threshold(j)=randi(10)+10;
    elseif timeout(j)==timeout_threshold(j) %If collision but timeout expired then go back
        %compute new path
        [new_index, new_waypoints, updated_graph] = compute_new_path(wp_index(j), waypoints{j}, graphe);
        if flag == 1 % on ne supprime pas l'arête du graphe si collision avec véhicule
            graphe = updated_graph; % information globale
        end
        waypoints{j} = new_waypoints;
        wp_index(j) = new_index;
        timeout(j)=0;
        carStopped(j)=0;
    else %If timeout not expired then stop
        timeout(j)=timeout(j)+1;
        carStopped(j)=1;
    end
else
    carStopped(j)=0;
end
previousFlags(j) = flag;

end