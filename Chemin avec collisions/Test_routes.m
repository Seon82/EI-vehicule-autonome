clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

%Initial node posisitons for cars, one integer = one car
CAR_NODE_POSITION = [0];

%Initial speeds, one integer = one speed
CAR_SPEED = [2];

%Create cars
%cars = addCars(scenario, CAR_NODE_POSITION);

car = vehicle(scenario, 'ClassID', 1, 'Position', [0 0 0]);

waypoints = [1 2 3 4 5 12 11 10 18 30 29 28 27 17 16 15 8 7 6 13 23 24 21 22 25 26 20 19 11];
i = 1;
vitesse =2;

plot(scenario);
while advance(scenario)
    [next_Position, next_Yaw, reached] = motionRectiligne(car,noeuds(waypoints(i)),vitesse,Ts);
    car.Yaw = next_Yaw;
    car.Position = next_Position;
    if reached==1
        i = i+1;
    end
    updatePlots(scenario)
    pause(0.01)
end