function [new_index, new_waypoints, updated_graph]= compute_new_path(wp_index, waypoints, graphe)
updated_graph = rmedge(graphe, waypoints(wp_index-1), waypoints(wp_index));
new_waypoints = [waypoints(1:max(1, wp_index-2)), shortest_path(updated_graph, waypoints(wp_index-1), waypoints(end))];
new_index = wp_index - 1;