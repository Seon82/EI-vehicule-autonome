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

package_delivered=0;
iteration=1;

plot(scenario);
while advance(scenario)
    for j=1:length(cars)
        %get_collision()
        %get_flag()
        %collisionHandler
        
        if flags(j,iteration)==1
            if iteration<1 || flags(j,iteration-1)==0 %If initial collision imminent then stop
                wait=0;
            elseif wait==10 %If collision but timeout expired then go back
                [new_index, new_waypoints, updated_graph] = compute_new_path(wp_index(j), waypoints{j}, graphe);
                graphe = updated_graph; % information globale
                waypoints{j} = new_waypoints;
                wp_index(j) = new_index;
                wait=0;
            else %If timeout not expired then stop
                wait=wait+1;
                wait;
            end
        end
        
        [cars, wp_index] = moveCars(cars,j,waypoints,wp_index,noeuds,CAR_SPEED,Ts, compute_new_path);
    end
    
    iteration=iteration+1;
    updatePlots(scenario)
    pause(0.01)
end