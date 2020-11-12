function [distance_mat] = get_distance(actors, alpha, range)
distance_mat = zeros(length(actors)) + inf;
for i=1:length(actors)
    car = actors(i);
    A = car.Position(1:2);
    B = range * [cosd(car.Yaw - alpha), sind(car.Yaw - alpha)] + A;
    C = range * [cosd(car.Yaw + alpha), sind(car.Yaw + alpha)] + A;
    % Calcul des vecteurs
    v0 = C - A;
    v1 = B - A;
    %Calcul des produits scalaires
    dot00 = dot(v0, v0);
    dot01 = dot(v0, v1);
    dot11 = dot(v1, v1);
    % On vérifie si un des autres objects est dans le triangle de détection
    for j=1:length(actors)
        if i ~= j
            other_car = actors(j);
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
            end
        end
    end
end
end




            
            
            
            

