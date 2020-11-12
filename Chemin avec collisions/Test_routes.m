clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

CAR_SPEED = [2 3]; %Initial speeds, one integer = one speed

%Create delivery point
delivery_node = 22;
delivery_point = vehicle(scenario,'ClassID',2,'Length',2,'Width',2, 'Position', noeuds(delivery_node), 'PlotColor', 'r');
%Create obstacle
obstacle_node = 13;
obstacle = vehicle(scenario,'ClassID',3,'Length',2,'Width',2, 'Position', noeuds(obstacle_node), 'PlotColor', 'k');

waypoints1 = shortestpath(graphe, 1, delivery_node);
waypoints2 = [3 8 7 2 3 8 7 2];
waypoints = {waypoints1 waypoints2};

%Create cars
cars = createCars(scenario, waypoints, noeuds);
nb_cars = length(cars);

package_delivered=0;
wp_index = zeros(1,nb_cars)+1;
previousFlags = zeros(1,nb_cars);
carStopped = zeros(1,nb_cars);
timeout = zeros(1,nb_cars);
timeout_threshold = zeros(1,nb_cars);

plot(scenario);
while advance(scenario)
    for j=1:nb_cars
        distanceMat = get_distance(cars, obstacle, 10, 100);
        flag=get_flag(j,nb_cars,distanceMat,5);
                
        [timeout, graphe, carStopped, previousFlags, wp_index, waypoints, timeout_threshold] = collisionHandler(...
            flag, j, previousFlags, carStopped, timeout, timeout_threshold, wp_index, waypoints, graphe);

        if carStopped(j)==0
            [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts);
        end
    end
    
    updatePlots(scenario)
    pause(0.01)
end