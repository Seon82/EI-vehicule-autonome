clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

CAR_SPEED = [2 3]; %Initial speeds, one integer = one speed
DISTANCE_STOP = 10;
CONDITIONS_RADIO_DEGRADEES = 0;

%Create delivery point
delivery_node = 22;
delivery_point = vehicle(scenario,'ClassID',2,'Length',2,'Width',2, 'Position', noeuds(delivery_node), 'PlotColor', 'r');
%Create obstacle
obstacle_coord = (noeuds(6)+noeuds(13))/2;
obstacle = vehicle(scenario,'ClassID',3,'Length',2,'Width',2, 'Position', obstacle_coord, 'PlotColor', 'k');

waypoints = {shortestpath(graphe, 1, delivery_node) [5]};

if CONDITIONS_RADIO_DEGRADEES == 1
    for i=1:nb_cars
        graphes{i} = graphe;
    end
end

%Create cars
cars = createCars(scenario, waypoints, noeuds);
nb_cars = length(cars);

vehicules_livraison = zeros(1, nb_cars);
vehicules_livraison(1) = 1;
package_delivered = zeros(1, nb_cars);
wp_index = zeros(1,nb_cars)+1;
previousFlags = zeros(1,nb_cars);
carStopped = zeros(1,nb_cars);
timeout = zeros(1,nb_cars);
timeout_threshold = zeros(1,nb_cars);

plot(scenario);
while advance(scenario)
    for j=1:nb_cars
        if CONDITIONS_RADIO_DEGRADEES==1
            graphe = graphes{j};
        end
        distanceMat = get_distance(cars, obstacle, 10, 100);
        flag=get_flag(j,nb_cars,distanceMat,DISTANCE_STOP);
                
        [timeout, graphe, carStopped, previousFlags, wp_index, waypoints, timeout_threshold] = collisionHandler(...
            flag, j, previousFlags, carStopped, timeout, timeout_threshold, wp_index, waypoints, graphe);

        if carStopped(j)==0
            if wp_index(j) > length(waypoints{j})
               [waypoints{j}, package_delivered(j)] = get_new_waypoints(j, waypoints{j}, vehicules_livraison, package_delivered(j), graphe);
               wp_index(j) = 1;
            end
            [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts);
        end
    end
    
    updatePlots(scenario)
    pause(0.01)
end