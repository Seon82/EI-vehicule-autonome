function [distance_mat, collision_mat] = get_collisions(cars, alpha, range, distance_alerte)
distance_mat = zeros(length(cars)) + inf;
collision_mat = zeros(length(cars));
for i=1:length(cars)
    A = cars(i).Position(1:2);
    B = range * [cosd(car.Yaw - alpha), sind(car.Yaw - alpha)];
    C = range * [cosd(car.Yaw + alpha), sind(car.Yaw + alpha)];
    % Calcul des vecteurs
    v0 = C - A;
    v1 = B - A;
    %Calcul des produits scalaires
    dot00 = dot(v0, v0);
    dot01 = dot(v0, v1);
    dot11 = dot(v1, v1);
    % Calcul des coordonnées 
    for j=1:length(cars)
        if i ~= j
            car = cars(i);
            other_car = cars(j);
            P = other_car.Position(1:2);
            v2 = P - A;
            dot02 = dot(v0, v2);
            dot12 = dot(v1, v2);
            %coordonnées barycentriques
            invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            if (u >= 0) && (v >= 0) && (u + v < 1) % Si P est dans le triangle ABC
               distance_mat(i,j) = norm(car.Position - other_car.Position);
               collision_mat(i,j) = (distance_mat(i,j) > distance_alerte);
            end
        end
    end
end
end




            
            
            
            

