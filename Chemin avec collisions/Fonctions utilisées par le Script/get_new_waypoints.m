function [new_waypoints, package_delivered] = get_new_waypoints(index_vehicule, waypoints, vehicules_livraison, package_delivered, graphe) 
    if vehicules_livraison(index_vehicule)==1
        if package_delivered == 0
            new_waypoints = graphe.shortestpath(waypoints(end), waypoints(1));
            package_delivered = 1;
        else
            disp("Terminé")
        end
    else
        new_waypoints = shortestpath(graphe, waypoints(end), randi(30));
    end
end


