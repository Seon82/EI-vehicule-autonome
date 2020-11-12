clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

CAR_SPEED = [2, 3]; %Initial speeds, one integer = one speed

waypoints1 = [1 2 3 4 5 12 11 10 18 30 29 28 27 17 16 15 8 7 6 13 23 24 21 22 25 26 20 19 11];
waypoints2 = [3 8 7 2 3 8 7 2];
waypoints = {waypoints1 waypoints2};

%Create cars
cars = createCars(scenario, waypoints, noeuds);

wp_index = zeros(1,length(cars))+1;

plot(scenario);
while advance(scenario)
    for j=1:length(cars)
        [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts);
    end
    updatePlots(scenario)
    pause(0.01)
end