function [node_id] = getNodeId(i, j, max_i)
    node_id = j*(max_i+1) + i + 1; % node ids can't be 0 so +1
end

