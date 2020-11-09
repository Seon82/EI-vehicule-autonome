%function to calculate the relative distance and the stop signal based on
%the sensor signal measurements
function [relative_distance,flag_stop] = distancesensor(vehicle2)
relative_distance = sqrt((vehicle2.Measurement(1))^2+(vehicle2.Measurement(2))^2+(vehicle2.Measurement(3))^2);
if relative_distance < 5 %if less than <5m the vehicle must stop.
    flag_stop =1; 
else
    flag_stop=0;
end
