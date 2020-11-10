function cars = addCars(scenario, positions)

cars=[];
for i=1:length(positions)
    car = vehicle(scenario,'ClassID',1,'Position',positions(i,:)*20);
    cars=[cars,car];
end

end