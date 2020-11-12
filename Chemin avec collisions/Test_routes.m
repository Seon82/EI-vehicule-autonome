clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

CAR_SPEED = [2 2 3]; %Initial speeds, one integer = one speed
DISTANCE_STOP = 10;
CONDITIONS_RADIO_DEGRADEES = 0;

% Create delivery points
delivery_nodes = [22, 11];
generate_delivery = @(node) vehicle(scenario,'ClassID',2,'Length',2,'Width',2, 'Position', noeuds(node), 'PlotColor', 'r');
delivery_points = arrayfun(generate_delivery, delivery_nodes);

% Create trajectories
waypoints1 = shortestpath(graphe, 1, delivery_nodes(1));
waypoints2 = shortestpath(graphe, 2, delivery_nodes(2));
waypoints = {waypoints1 waypoints2 [5]};

% Create cars
cars = createCars(scenario, waypoints, noeuds);
nb_cars = length(cars);


% Setup deliveries
vehicules_livraison = zeros(1, nb_cars);
vehicules_livraison(1:2) = 1;
package_delivered = zeros(1, nb_cars);

% Setup obstacles
obstacle_coord = (noeuds(6)+noeuds(13))/2;
obstacle = vehicle(scenario,'ClassID',3,'Length',2,'Width',2, 'Position', obstacle_coord, 'PlotColor', 'k');

% Misc variables
wp_index = zeros(1,nb_cars)+1;
previousFlags = zeros(1,nb_cars);
carStopped = zeros(1,nb_cars);
timeout = zeros(1,nb_cars);
timeout_threshold = zeros(1,nb_cars);
deleted_cars = zeros(1,nb_cars);

if CONDITIONS_RADIO_DEGRADEES == 1
    for i=1:nb_cars
        graphes{i} = graphe;
    end
end

% Start main loop
plot(scenario);
while advance(scenario)
    for j=1:nb_cars
        if deleted_cars(j)==0
            if CONDITIONS_RADIO_DEGRADEES==1
                graphe = graphes{j};
            end
            distanceMat = get_distance(cars, obstacle, deleted_cars, 10, 100);
            flag=get_flag(j,nb_cars,distanceMat,DISTANCE_STOP);

            [timeout, graphe, carStopped, previousFlags, wp_index, waypoints, timeout_threshold] = collisionHandler(...
                flag, j, previousFlags, carStopped, timeout, timeout_threshold, wp_index, waypoints, graphe);

            if carStopped(j)==0
                if wp_index(j) > length(waypoints{j})
                   [waypoints{j}, package_delivered(j), deleted_cars(j)] = get_new_waypoints(waypoints{j}, vehicules_livraison(j), package_delivered(j), deleted_cars(j), graphe);
                   if deleted_cars(j) % Hide car
                       cars(j).Height = 0.01;
                       cars(j).Width = 0.01;
                       cars(j).PlotColor = "#e6e6e6";
                       cars(j).Yaw = 0;
                       break
                   end
                   if package_delivered(j)
                       delivery_points(sum(vehicules_livraison(1:j))).PlotColor = 'g';
                   end
                   wp_index(j) = 1;
                end
                [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts);
            end
        end
    end
    
    updatePlots(scenario)
    pause(0.01)
end