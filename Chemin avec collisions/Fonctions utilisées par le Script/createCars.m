function cars = createCars(scenario, waypoints, noeuds)

cars=[];
for i=1:length(waypoints)
    waypoint=waypoints{1,i};
    car = vehicle(scenario,'ClassID',1,'Position',noeuds(waypoint(1)));
    cars=[cars,car];
end

end