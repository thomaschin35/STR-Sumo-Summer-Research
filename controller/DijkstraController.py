from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class DijkstraPolicy(RouteController):

    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info, vehicle_states, step):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            # print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
            decision_list = []
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
            visited = {} # map of visited edges
            current_edge = vehicle.current_edge

            current_distance = self.connection_info.edge_length_dict[current_edge]
            unvisited[current_edge] = current_distance
            path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    new_distance = current_distance + edge_length
                    if new_distance < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = new_distance
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)
                        #print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

                visited[current_edge] = current_distance
                del unvisited[current_edge]
                if not unvisited:
                    break
                if current_edge==vehicle.destination:
                    break

                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
                #print('{}:{}------------'.format(current_edge, current_distance))
            #current_edge = vehicle.current_edge
            if len(path_lists[vehicle.destination]) == 0: 
                state = self.getState(vehicle.current_edge, vehicle.destination, vehicle.vehicle_id, step)
            else: 
                state = self.getState(vehicle.current_edge, vehicle.destination, vehicle.vehicle_id, step, path_lists[vehicle.destination][0])
            vehicle_states.append(state)
            # print(vehicle.vehicle_id)
            # print(path_lists[vehicle.destination])

            for direction in path_lists[vehicle.destination]:
                decision_list.append(direction)
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
            # print(local_targets[vehicle.vehicle_id])
        return local_targets, vehicle_states
    
    def getState(self, edge_now, destination, vid, step, right_direction="NaN"):
        en = edge_now
        state = []
        #include final location?
        state.append(step)
        state.append(vid)
        state.append(self.connection_info.edge_index_dict[en]) #getting the current edge index
        state.append(right_direction)
        state.append(self.connection_info.edge_index_dict[destination])
        # state.append()
        for c in self.direction_choices:
            if c in self.connection_info.outgoing_edges_dict[en].keys():
                state.append(1)
                # 1 stands for the edge is available for being chose
            else:
                state.append(0)
                # 0 means this action cannot be chosen.
        # put the congestion ratio of all edges into the state.
        for edge in self.connection_info.edge_list:
            car_num = traci.edge.getLastStepVehicleNumber(edge)
            density = car_num / self.connection_info.edge_length_dict[edge]
            state.append(density)

        state = np.reshape(state, [1, len(state)]) #2D array with 1 row and as many columns as state
        #state = [[edge index, 1, 0 ,0 ,1, 1, 1, density, ... (density of every other edge)]
        # print(state.shape)
        return state
    
