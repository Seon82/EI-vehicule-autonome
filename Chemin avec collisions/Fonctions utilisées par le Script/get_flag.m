function flag = get_flag(car_index, distance_mat, distance_seuil)
flag = any(distance_mat(car_index,:)<=distance_seuil);
end