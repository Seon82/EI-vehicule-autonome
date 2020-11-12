clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

CAR_SPEED = [2 3]; %Initial speeds, one integer = one speed

%Create delivery point
delivery_node = 22;
delivery_point = vehicle(scenario,'ClassID',2,'Length',2,'Width',2, 'Position', noeuds(delivery_node), 'PlotColor', 'r');
[waypoints1,~] = shortestpath(graphe, 1, delivery_node);

waypoints2 = [3 8 7 2 3 8 7 2];
waypoints = {waypoints1 waypoints2};

%Create cars
cars = createCars(scenario, waypoints, noeuds);

wp_index = zeros(1,length(cars))+1;


plot(scenario);
while advance(scenario)
    for j=1:length(cars)
        %get_collision()
        [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts);
    end
    updatePlots(scenario)
    pause(0.01)
end