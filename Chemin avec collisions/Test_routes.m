clear all;
close all;

[scenario, noeuds, graphe] = createRoads();
Ts = 0.1;
scenario.SampleTime = Ts;

%Initial node posisitons for cars, one integer = one car
CAR_NODE_POSITION = [1, 3];

%Initial speeds, one integer = one speed
CAR_SPEED = [2, 3];

%Create cars
%cars = addCars(scenario, CAR_NODE_POSITION);

car = vehicle(scenario, 'ClassID', 1, 'Position', noeuds(1));
car2= vehicle(scenario, 'ClassID', 1, 'Position', noeuds(3));
cars= [car, car2];

waypoints1 = [1 2 3 4 5 12 11 10 18 30 29 28 27 17 16 15 8 7 6 13 23 24 21 22 25 26 20 19 11];
waypoints2 = [3 8 7 2];
waypoints = [waypoints1 waypoints2];
i = 1;

plot(scenario);
while advance(scenario)
    for j=1:length(cars)
        [next_Position, next_Yaw, reached] = motionRectiligne(cars(j),noeuds(waypoints(i)),CAR_SPEED(j),Ts);
        cars(j).Yaw = next_Yaw;
        cars(j).Position = next_Position;
        if reached==1
            i = i+1;
        end
    end
    updatePlots(scenario)
    pause(0.01)
end