from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class TestPolicy(RouteController):
    def __init__(self, connection_info):
        super().__init__(connection_info)

    
    
    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            # print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
            decision_list = []
            unvisited = {
                edge: 1000000000 for edge in self.connection_info.edge_list
            }  # map of unvisited edges
            visited = {}  # map of visited edges
            current_edge = vehicle.current_edge
            print(current_edge)
            current_distance = self.distance_with_traffic(current_edge) #altered distance to account for traffic
            # current_distance = self.connection_info.edge_length_dict[current_edge]
            
            unvisited[current_edge] = current_distance
            path_lists = {
                edge: [] for edge in self.connection_info.edge_list
            }  # stores shortest path to each edge using directions
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                for (
                    direction,
                    outgoing_edge,
                ) in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    # edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    edge_length = self.distance_with_traffic(outgoing_edge)
                    new_distance = current_distance + edge_length
                    if new_distance < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = new_distance
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)
                        # print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

                visited[current_edge] = current_distance
                del unvisited[current_edge]
                if not unvisited:
                    break
                if current_edge == vehicle.destination:
                    break
                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(
                    possible_edges, key=lambda x: x[1]
                )[0]
                # print('{}:{}------------'.format(current_edge, current_distance))
            # current_edge = vehicle.current_edge

            for direction in path_lists[vehicle.destination]:
                best_direction = direction
                # Lazy attempt at analyzing traffic data

                # best_edge = vehicle.current_edge
                # # print(best_edge)
                # # print(self.connection_info.outgoing_edges_dict[vehicle.current_edge])
                # for dir, edge in self.connection_info.outgoing_edges_dict[vehicle.current_edge].items() :
                #     # print(edge)
                #     if traci.edge.getLastStepVehicleNumber(edge) < traci.edge.getLastStepVehicleNumber(best_edge):
                #         best_direction = dir
                #         best_edge = edge

                decision_list.append(best_direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(
                decision_list, vehicle
            )
        return local_targets
    
    # Calculating the distance of an edge an altering it based on traffic data 
    def distance_with_traffic(self, edge) : 
        current_distance = self.connection_info.edge_length_dict[edge]
        traffic_density = self.calculate_traffic_density(edge)
        
        congestion_factor = 1 + (traffic_density * 0.1)
        # For future could use queue information, how long vehicles waiting at an edge for
        return current_distance * congestion_factor

    # Calculate traffic density for a specific road segment
    def calculate_traffic_density(self, segment_id):
        vehicle_ids = traci.edge.getLastStepVehicleIDs(segment_id)
        num_vehicles = len(vehicle_ids)
        segment_length = self.connection_info.edge_length_dict[segment_id]
        density = num_vehicles / segment_length
        return density
    
    # def calculate_waiting_time(self, edge):
    #     vehicle_ids = traci.edge.getLastStepVehicleIDs(edge)
    #     average_waiting_time = 0

            


    # Detect traffic queues for a specific intersection
    def detect_traffic_queue(intersection_id):
        queue_length = traci.junction.getWaitingTime(intersection_id)
        return queue_length


