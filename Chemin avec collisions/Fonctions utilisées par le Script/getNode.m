function [coords] = getNode(node_id, max_i)
node_id = node_id-1;
coords(1) = mod(node_id, max_i+1);
coords(2) = (node_id - coords(1))/(max_i+1);
end