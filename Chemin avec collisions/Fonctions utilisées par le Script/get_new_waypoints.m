function [new_waypoints, package_delivered, deleted_cars] = get_new_waypoints(waypoints, vehicule_livraison, package_delivered, deleted_cars, graphe) 
    if vehicule_livraison==1
        if package_delivered == 0
            new_waypoints = graphe.shortestpath(waypoints(end), waypoints(1));
            package_delivered = 1;
            deleted_cars = 0;
        else
            deleted_cars = 1;
            new_waypoints = [waypoints(end)];
            package_delivered = 1;
            disp("Terminé")
        end
    else
        new_waypoints = shortestpath(graphe, waypoints(end), randi(30));
    end
end


