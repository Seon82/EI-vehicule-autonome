function flag = get_flag(car_index, num_cars, distance_mat, distance_seuil)
% flag = 0 if no close obstacle is detected, 2 if an unmoving obstacle is
% detected, 1 if a car is detected.
   index = find(distance_mat(car_index,:)<=distance_seuil, 1);
   if isempty(index)==1
       flag = 0;
   elseif index > num_cars
       flag = 1;
   else
       flag = 2;
   end
end