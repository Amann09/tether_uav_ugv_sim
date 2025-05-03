import networkx as nx
import matplotlib.pyplot as plt
import random
import math
from random import uniform
 
import matplotlib.animation as animation 
import numpy as np
from itertools import combinations
import itertools
from py2opt.routefinder import RouteFinder

n = int(input("Enter grid size: "))

# Create a complete graph with n*n nodes
G = nx.complete_graph(n*n)

# Generate positions for each node to form an 8x8 grid layout
my_pos = {i: (i % n, i // n) for i in range(n*n)}

# Add some randomness to the positions
for node, pos in my_pos.items():
    # my_pos[node] = (pos[0] + random.uniform(-0.05, 0.05), pos[1] + random.uniform(-0.05, 0.05))
    my_pos[node] = (pos[0], pos[1])

# Draw the graph using the generated positions
# plt. figure(figsize=(200, 200))
nx.draw(G, pos=my_pos, with_labels=True, node_size=200, node_color='skyblue', edge_color='white')
# plt.show()

# -------------------------------------------------------------------------------------------

def eucl_dist(x1,y1,x2,y2):
    return math.sqrt( (x1-x2)**2 + (y1-y2)**2 )

list_of_Edges = list(G.edges)
list_of_Nodes = list(G.nodes)

# print(list_of_Edges)
# print(list_of_Nodes)
# print(f"Original Length of List_of_Edges: {len(list_of_Edges)}")
# print(f"Original Length of List_of_Nodes: {len(list_of_Nodes)}")


obs_coordinates = [(8, 12), (10, 6)]
# obs_coordinates = []

def calculate_neighbors(coord):
    x, y = coord[0], coord[1]
    neighbors = [(x-1, y-1), (x-1, y), (x-1, y+1),
                 (x, y-1), (x, y), (x, y+1),
                 (x+1, y-1), (x+1, y), (x+1, y+1)]
    return neighbors

obs_coordinates_step2 = []
for coordinate in obs_coordinates:
    obs_coordinates_step2.append(calculate_neighbors(coordinate))

print(obs_coordinates_step2)
final_obs = []
for x in obs_coordinates_step2:
    final_obs = final_obs + x 

# print(final_obs)

coordinates = []
for i in range(0, n):
    for j in range(0, n):
        coordinates.append((i, j))

# print(coordinates)

my_dict = {i: coordinates[i] for i in range(len(coordinates))}
# print("Dictionary:", my_dict)

all_nodes = list(my_dict.keys())
all_coordinates = list(my_dict.values())

obs_nodes = []
for coordinate in final_obs:
    obs_nodes.append(all_coordinates.index(coordinate))

print(f"Obstacle Nodes: {obs_nodes}")

walkable_nodes = [node for node in list_of_Nodes if node not in obs_nodes]

# for i in range(len(my_dict)):
#     print(f"{all_nodes[i]} : {all_coordinates[i]}")

# dist_bet_all_nodes = []
# for i,j in G.edges:
#   (x1,y1) = my_pos[i]
#   (x2,y2) = my_pos[j]
#   dist_bet_all_nodes.append(eucl_dist(x1,y1,x2,y2))

# # list_of_Edges = list(G.edges)
# distance_dict = { k:v for (k,v) in zip(list_of_Edges, dist_bet_all_nodes)}
# # print(distance_dict)
# edges_to_remove = [edge for edge, distance in distance_dict.items() if distance > 1.5]
# G.remove_edges_from(edges_to_remove)

# # list_of_Edges = G.edges
# # # print(list_of_Edges)
# # print(f"Length of List_of_Edges after removing edges greater than 1.5 units: {len(list_of_Edges)}")


# Giving the colors to walkable nodes and obstacle nodes
color_map = []
color_map2 = []
for node in list_of_Nodes:
  if node not in obs_nodes:
    color_map.append('skyblue')
    color_map2.append('skyblue')
  else:
    color_map.append('red')

obs_nodes = set(obs_nodes)
new_list_of_Edges = [edge for edge in list_of_Edges if not any(num in obs_nodes for num in edge)]

plt. figure(figsize=(200, 200))
# nx.draw_networkx(G, pos=my_pos, nodelist=walkable_nodes, node_color=color_map2, edgelist=new_list_of_Edges, edge_color='white')
nx.draw_networkx(G, pos=my_pos, nodelist=list_of_Nodes, node_color=color_map, edgelist=new_list_of_Edges, edge_color='white')
plt.show()

# -----------------------------------------------------------------------------------------------

def eucl_dist(x1,y1,x2,y2):
    return math.sqrt( (x1-x2)**2 + (y1-y2)**2 )

H = G.copy() # copy our main graph, just to be safe

dist_bet_all_nodes = []
for i,j in G.edges:
  (x1,y1) = my_pos[i]
  (x2,y2) = my_pos[j]
  dist_bet_all_nodes.append(eucl_dist(x1,y1,x2,y2))

# print(f"Distance between all the Nodes: {dist_bet_all_nodes}")
print(f"Length of Distance between all the Nodes: {len(dist_bet_all_nodes)}")

distance_dict = { k:v for (k,v) in zip(list_of_Edges, dist_bet_all_nodes)}
print(f"Distance Dict: {distance_dict}")


edges_to_remove = [edge for edge, distance in distance_dict.items() if distance > 1.5]

G.remove_edges_from(edges_to_remove)

list_of_Edges = G.edges
print(f"Length of List_of_Edges after removing edges greater than 1.5 units: {len(list_of_Edges)}")

''' removing the edges from Graph '''
edges_to_drop = [edge for edge in list_of_Edges if any(num in obs_nodes for num in edge)]
print(f"EDGES TO DROP: \n{edges_to_drop}")

my_dict = {key: [] for key in list(obs_nodes)}

# Populate the dictionary with values based on list_1
for edge in edges_to_drop:
    if edge[0] in my_dict:
        my_dict[edge[0]].append(edge[1])
    if edge[1] in my_dict:
        my_dict[edge[1]].append(edge[0])

sorted_dict = {k: my_dict[k] for k in sorted(my_dict)}

print(f"SORTED DICT: {sorted_dict}")

## removing the edges that is connected to osbtacle nodes.
# G.remove_edges_from(edges_to_drop)

G.remove_nodes_from(list(obs_nodes))

list_of_Edges = list(G.edges)
list_of_Nodes = list(G.nodes)

print(f"Length of List_of_Edges after removing obstacle edges: {len(list_of_Edges)}")
print(f"Length of List_of_Nodes after removing obstacle nodes: {len(list_of_Nodes)}")


for edge in list(distance_dict.keys()):
  if (edge in edges_to_remove) or (edge in edges_to_drop):
    del distance_dict[edge]

## ---------------------------------------------------------------------------------------

## Drawing the graph after removing the edges
plt.figure(figsize=(200, 200))
plt.title('Graph after removing Edges')
nx.draw(G, pos=my_pos, with_labels=True, node_size=200, node_color='skyblue')
plt.show()

## ---------------------------------------------------------------------------------------

# Finding minimum spanning tree
T = nx.minimum_spanning_tree(G, weight='length')
# plt. figure(figsize=(200, 200))

list_of_Edges_for_T = list(T.edges)
# print(list_of_Edges_for_T)

obs_nodes = set(obs_nodes)
new_list_of_Edges_for_T = [edge for edge in list_of_Edges_for_T if not any(num in obs_nodes for num in edge)]

nx.draw_networkx(T, pos=my_pos, nodelist=list_of_Nodes, node_color=color_map2, edgelist=new_list_of_Edges_for_T , edge_color='black')
# plt.show()

## ---------------------------------------------------------------------------------------

# Identify the odd-degree nodes
odd_degree_nodes = [ i for i in T.nodes if T.degree(i) % 2 ]
node_colors = [ T.degree(i) % 2 for i in T.nodes ]
# plt. figure(figsize=(200, 200))
nx.draw(T, pos=my_pos, node_color=node_colors, with_labels=True)
# plt.show()

## ---------------------------------------------------------------------------------------

print(f"Length of G.edges : {len(G.edges)}")
print(G.edges)


# Finding a minimum-cost perfect matching over the odd-degree nodes\
for i,j in G.edges:
    (x1,y1) = my_pos[i]
    (x2,y2) = my_pos[j]
    G.edges[i,j]['neg_length'] = - eucl_dist(x1,y1,x2,y2)
       

matching = nx.max_weight_matching( G.subgraph(odd_degree_nodes), maxcardinality=True, weight='neg_length')

# Drawing the matching
plt. figure(figsize=(200, 200))
nx.draw(G.edge_subgraph(matching),pos=my_pos, with_labels=True)
plt.show()

# create a multigraph with edge_set = (spanning tree edges) + (matching)
M = nx.MultiGraph()

# M.add_nodes_from(range(n*n))
M.add_nodes_from(walkable_nodes)

M.add_edges_from(T.edges())
M.add_edges_from(matching)

# print(M.edges())
# print("M has this many edges =",M.number_of_edges())
print(f"Multigraph M is Eulerian: {nx.is_eulerian(M)}")

H = nx.eulerize(M)
print(f"Multigraph H is Eulerian: {nx.is_eulerian(H)}")

# find an Eulerian cycle of the multigraph
initial_tour = list ( nx.eulerian_circuit(H, source=0) )


# take shortcuts (avoid repeated nodes)
tour = [ 0 ]
for (i,j) in initial_tour:
    if j not in tour:
        tour.append(j)

# print(tour)
# print(len(tour))

tour_edges = [ (tour[i-1],tour[i]) for i in range(len(tour)) ]
lengthOfTour = len(tour)
tour_nodes = list(tour)
print(f"TOUR: {tour_nodes}")
print(f"TOUR EDGES: {tour_edges}")

# draw the tour
plt.figure(figsize=(200, 200))
plt.title('Original Tour')
nx.draw(G.edge_subgraph(tour_edges), pos=my_pos, with_labels=True)
plt.show()

tour_edges = tour_edges[1:]
def edges_with_distance(tour_nodes):
  t = []
  for node1, node2 in itertools.combinations(tour_nodes, 2):
      (x1, y1) = my_pos[node1]
      (x2, y2) = my_pos[node2]
      distance = eucl_dist(x1,y1,x2,y2)
      t.append((node1, node2, distance))
  return t

tour_edges_withDist = edges_with_distance(tour_nodes)
print(tour_edges_withDist)

W = nx.Graph()
W.add_nodes_from(tour_nodes)
W.add_weighted_edges_from(tour_edges_withDist)
dist_mat = nx.floyd_warshall_numpy(W)
print(dist_mat)
route_finder = RouteFinder(dist_mat, tour_nodes, iterations=5)
best_distance, best_route = route_finder.solve()
print(best_distance)
print(best_route)
optimized_tour_edges = [ (best_route[i-1], best_route[i]) for i in range(len(best_route)) ]
optimized_tour_edges = optimized_tour_edges[1:]

# draw the tour
plt.figure(figsize=(200, 200))
plt.title('Optimized Tour')
nx.draw(W.edge_subgraph(optimized_tour_edges), pos=my_pos, with_labels=True)
plt.show()

waypoints = []
for node in best_route:
  (x, y) = my_pos[node]
  waypoints.append((x, y))

print(waypoints)


# Create a function to update the plot with the current tour
def update_plot(i):
    plt.clf()
    plt.title(f'Step {i+1}')
    current_tour = best_route[:i+1]
    
    nx.draw(G, pos=my_pos, node_color='blue', node_size=100, edge_color='white')
    nx.draw_networkx_nodes(G, pos=my_pos, nodelist=current_tour, node_color='red', node_size=100, edgecolors=None)
    if len(current_tour) > 1:
      list_of_currentEdges = [(current_tour[i], current_tour[i + 1]) for i in range(len(current_tour) - 1)]
      nx.draw_networkx_edges(G, pos=my_pos, edgelist=list_of_currentEdges, edge_color='skyblue')
    else:
      nx.draw_networkx_edges(G, pos=my_pos, edgelist=list(combinations(current_tour, 2)), edge_color='red')

# Create the plot
plt.ion()
fig = plt.figure(figsize=(200, 200))
# ani = animation.FuncAnimation(fig, update_plot, frames=len(tour_nodes), repeat=False)
for i in range(len(best_route)):
  update_plot(i)
  plt.pause(0.2)
  plt.show()  

plt.ioff()
plt.show()

best_route_drone = [x for x in best_route if best_route.index(x)%2 == 0]
best_route_rover = [x for x in best_route if best_route.index(x)%2 != 0]

# Create a function to update the plot with the current tours of both drone and rover
def update_plot_for_two(i):
    plt.clf()
    plt.title(f'Step {i+1}')

    # Get the current state of the tours for both drone and rover
    current_tour_drone = best_route_drone[:i+1]
    current_tour_rover = best_route_rover[:i+1]

    # Draw the base graph
    nx.draw(G, pos=my_pos, node_color='blue', node_size=100, edge_color='white')

    # Draw the nodes and edges for the drone's route
    nx.draw_networkx_nodes(G, pos=my_pos, nodelist=current_tour_drone, node_color='red', node_size=100, edgecolors=None)
    if len(current_tour_drone) > 1:
        list_of_currentEdges_drone = [(current_tour_drone[i], current_tour_drone[i + 1]) for i in range(len(current_tour_drone) - 1)]
        nx.draw_networkx_edges(G, pos=my_pos, edgelist=list_of_currentEdges_drone, edge_color='skyblue')
    else:
        nx.draw_networkx_edges(G, pos=my_pos, edgelist=list(combinations(current_tour_drone, 2)), edge_color='red')

    # Draw the nodes and edges for the rover's route
    nx.draw_networkx_nodes(G, pos=my_pos, nodelist=current_tour_rover, node_color='green', node_size=100, edgecolors=None)
    if len(current_tour_rover) > 1:
        list_of_currentEdges_rover = [(current_tour_rover[i], current_tour_rover[i + 1]) for i in range(len(current_tour_rover) - 1)]
        nx.draw_networkx_edges(G, pos=my_pos, edgelist=list_of_currentEdges_rover, edge_color='orange')
    else:
        nx.draw_networkx_edges(G, pos=my_pos, edgelist=list(combinations(current_tour_rover, 2)), edge_color='green')

# Create the plot
plt.ion()
fig = plt.figure(figsize=(200, 200))
for i in range(max(len(best_route_drone), len(best_route_rover))):
    update_plot_for_two(i)
    plt.pause(0.2)
    plt.show()

plt.ioff()
plt.show()