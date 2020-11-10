function flagColi = detectCollision(vecControl,objectDetections,k)

for j=1:length(vecControl) %Loop with the number of controlled vehicles (with sensor)
    for i=1:length(objectDetections) %Loop with the number of detected objects
        if objectDetections{i}.ObjectClassID~=2 %pas une livraison
            [relativeDist(j,i,k),flagColi(j,i,k)] = distancesensor(objectDetections{i});
        else
            relativeDist(j,i,k) = inf;
            flagColi(j,i,k) = 0;
        end
    end
end

end