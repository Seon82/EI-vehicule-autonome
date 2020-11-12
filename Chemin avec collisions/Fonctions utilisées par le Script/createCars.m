function cars = createCars(scenario, CAR_NODE_POSITION, noeuds)

cars=[];
for i=1:length(CAR_NODE_POSITION)
    car = vehicle(scenario,'ClassID',1,'Position',noeuds(CAR_NODE_POSITION(i)));
    cars=[cars,car];
end

end