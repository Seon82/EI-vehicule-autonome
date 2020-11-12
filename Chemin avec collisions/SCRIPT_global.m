clear all
close all
% Construct a drivingScenario object.
Ts = 0.1; %sampling time
scenario = drivingScenario('SampleTime', Ts);

% Add all road segments
road_graph = graph;
L=20;
TAILLE_CARRE = 5;
for i=0:TAILLE_CARRE
    for j=0:TAILLE_CARRE 
        if j<TAILLE_CARRE
            roadCenters = [i*L j*L 0 ;i*L (j+1)*L 0];
            laneSpecification = lanespec(1);
            road(scenario, roadCenters, 'Lanes', laneSpecification);
            road_graph = addedge(road_graph, getNodeId(i, j, TAILLE_CARRE), getNodeId(i, j+1, TAILLE_CARRE));
        end
        if i<TAILLE_CARRE
            roadCenters = [i*L j*L 0 ;(i+1)*L j*L 0];
            laneSpecification = lanespec(1);
            road(scenario, roadCenters, 'Lanes', laneSpecification);
            road_graph = addedge(road_graph, getNodeId(i, j, TAILLE_CARRE), getNodeId(i+1, j, TAILLE_CARRE));
        end
    end
end



%add vehicle
%car = vehicle(scenario, 'ClassID', 1, 'Position', [0 0 0]);
% noeud_livraison = [max((randi(TAILLE_CARRE+1)-1),1) (randi(TAILLE_CARRE+1)-1) 0];
noeud_livraison = [2 3 0];
coord_livraison = noeud_livraison*L;
point_livraison = vehicle(scenario,'ClassID',2,'Length',2,'Width',2, 'Position', coord_livraison, 'PlotColor', 'r');
[waypoints,~] = shortestpath(road_graph, getNodeId(0,0, TAILLE_CARRE), getNodeId(noeud_livraison(1), noeud_livraison(2), TAILLE_CARRE));

% add obstacles
% noeud_obstacle = [max((randi(TAILLE_CARRE+1)-1),1) (randi(TAILLE_CARRE+1)-1) 0];
noeud_obstacle = [2 1.5 0];
vehicle(scenario,'ClassID',3,'Length',2,'Width',2, 'Position', noeud_obstacle*L, 'PlotColor', 'k');

% Add other vehicles

%Initial node posisitons for cars
CAR_NODE_POSITION = [0 0 0;
                1 1 0;
                1 0 0];

%Initial speeds
CAR_SPEED = [2 3 1];

cars = addCars(scenario, CAR_NODE_POSITION);

egoCar = cars(1);
passingCar1 = cars(2);
passingCar2 = cars(3);

waypoints1 = [1 1 0; 4 1 0 ; 4 4 0 ; 1 4 0 ; 1 1 0]*L;
index1 = 1;

waypoints2 = [1 0 0; 4 0 0 ; 1 0 0]*L;
index2 = 1;

% Controlled vehicles
vec_control = [egoCar,passingCar1, passingCar2];
%vec_control_degraded = [passingCar2];


%Create one variable to aggregate all detections into a structure for later use
allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {});

plot(scenario)

current_node = 1;
direction = 1;
avoiding_collision = 0;
reached = 0;
fin = 0;
Livraison_OK=0;
iteration=1;
timeout = 0;

while advance(scenario) && fin == 0
    
    %flag_coli = detectCollision(vec_control, objectDetections, k);      
    for j=1:length(vec_control) %Loop with the number of controlled vehicles (with sensor)
        
        % Sensor - local information
        % Generate the target poses of all actors relative to a certain vehicle
        poses = targetPoses(vec_control(j));
        %posesDegraded = targetPosses(vec_control_degraded);
        time  = scenario.SimulationTime;
        
        % Create all the sensors
        
        sensor = createSensorCamera(scenario,Ts);
        
        % Generate detections for the sensor
        laneDetections = []; %not used for now
        [objectDetections, numObjects, isValidTime] = sensor(poses, time);
        objectDetections = objectDetections(1:numObjects);
        
        % Aggregate all detections into a structure for later use
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections',   {laneDetections});
        
        % Get object detection informations
        
        for i=1:length(objectDetections) %Loop with the number of detected objects
            if objectDetections{i}.ObjectClassID~=2 %pas une livraison
                [relative_dist(j,i,iteration),flag_coli(j,i,iteration)] = distancesensor(objectDetections{i});
            else
                relative_dist(j,i,iteration) = inf;
                flag_coli(j,i,iteration) = 0;
            end
        end
    end
    
    
    iteration=iteration+1;
    

    for j=1:length(vec_control)
        if isempty(objectDetections) %Pas d'objets détectés
            if reached == 1
                if avoiding_collision == 1
                    avoiding_collision = 0;
                    road_graph = rmedge(road_graph, waypoints(current_node),waypoints(current_node + 1*direction));
                    if direction == 1 %Aller
                        [new_waypoints,~] = shortestpath(road_graph, waypoints(current_node), getNodeId(noeud_livraison(1), noeud_livraison(2), TAILLE_CARRE));
                        waypoints = [waypoints(1:current_node-1), new_waypoints];
                    else %retour
                        [new_waypoints,~] = shortestpath(road_graph, getNodeId(0,0, TAILLE_CARRE), waypoints(current_node));
                        waypoints = [new_waypoints, waypoints(current_node-1,end)];
                    end
                end
            %verification si point de livraison atteint ; la voiture retourne au
            %point de depart
            if norm(egoCar.Position-coord_livraison)<1e-3 | current_node == 0
               if Livraison_OK==0
                   Livraison_OK=1;
                   point_livraison.PlotColor = 'g';
                   disp('livré');
                   coord_livraison = [0 0 0];
                   direction = -1;
               else
                   disp('arrivé');
                   fin=1;
               end
            end
            current_node = current_node + 1*direction;
            end
            [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(1), Ts);
            
            
        else %Objets détectés dans le périmètre
            global_timout=0;
            for object=1:length(objectDetections) %loop with the number of detected objects
                if flag_coli(j,object,iteration-1)==1 %checking if collision
                    
                    %[next_position, next_Yaw, reached, timeout, avoiding_collision] = collisionHandler(iteration,...
                     %   flag_coli, j, object, current_node, direction, next_position, next_Yaw, reached,...
                      %  vec_control, waypoints, TAILLE_CARRE, L, CAR_SPEED, Ts, timeout);
                    
                    if iteration<2 || flag_coli(j,object,iteration-2)==0 %If initial collision imminent then stop
                        
                        wait=0;
                        
                    elseif wait==10 %If collision but timeout expired then go back
                        current_node = current_node - 1*direction;
                        [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(1), Ts);
                        avoiding_collision=1;
                        wait=0;
                        
                    else %If timeout not expired then stop
                        wait=wait+1;
                        wait;
                        
                    end
                        

                else %Object detected but no collision                    
                    if j==1
                        [next_position, next_Yaw, reached] = motionRectiligne(vec_control(j), getNode(waypoints(current_node), TAILLE_CARRE)*L, CAR_SPEED(1), Ts);
                        if reached == 1
                            current_node = current_node + 1*direction;
                        end
                        
                        wait=0;
                    end
                end
            end
        end
        
        %déplacement du véhicule
        vec_control(j).Position=next_position;
        vec_control(j).Yaw=next_Yaw;

        %Déplacement des véhicules passifs
        
        moveCar(passingCar1, waypoints1, index1, CAR_SPEED(2), Ts)
        moveCar(passingCar2, waypoints2, index2, CAR_SPEED(3), Ts)
        
    end

updatePlots(scenario)
pause(0.01)
    
end

