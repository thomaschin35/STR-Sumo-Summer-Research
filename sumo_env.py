
import numpy as np
import os
import sys
import random
import math
# from numba import jit, cuda

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from core.Util import *
from xml.dom.minidom import parse, parseString
from main import get_controlled_vehicles
from controller.DijkstraController import DijkstraPolicy
from controller.RouteController import RouteController

from sumolib import checkBinary
import traci
STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"

class SumoEnv:

    def __init__(self, number_of_vehicles, gui=False, label='default'):
        self.label = label
        self.sumo_binary = checkBinary('sumo-gui') if gui else checkBinary('sumo')
        # sumo_binary = checkBinary('sumo')#use this line if you do not want the UI of SUMO
        # parse config file for map file namez
        dom = parse("./configurations/myconfig.sumocfg")

        net_file_node = dom.getElementsByTagName('net-file')
        net_file_attr = net_file_node[0].attributes

        net_file = net_file_attr['value'].nodeValue
        init_connection_info = ConnectionInfo("./configurations/"+net_file)

        self.direction_choices = [STRAIGHT, TURN_AROUND, SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]
        self.connection_info = init_connection_info
        self.route_controller = DijkstraPolicy(init_connection_info)
        # self.vehicles_to_direct = [] #  the batch of controlled vehicles passed to make_decisions()
        self.vehicle_IDs_in_simulation = []
        self.deadlines_missed = []
        self.total_time = 0
        self.step = 0
        self.current_edges = {}
        self.number_vehicles = number_of_vehicles
        

        route_file_node = dom.getElementsByTagName('route-files')
        route_file_attr = route_file_node[0].attributes
        route_file = "./configurations/"+route_file_attr['value'].nodeValue
        vehicles = get_controlled_vehicles(route_file, init_connection_info, self.number_vehicles, 50)
        self.controlled_vehicles = vehicles

        #print the controlled vehicles generated
        for vid, v in vehicles.items():
            print("id: {}, destination: {}, start time:{}, deadline: {};".format(vid, \
                v.destination, v.start_time, v.deadline))
            
    
    def step_d(self, action, rewards):
        done = False
        # states = np.zeros((self.number_vehicles, (len(self.connection_info.edge_list) + 9)), dtype=np.float32)
        connection_info = self.connection_info
        threshold_traffic = 0.5
        positions = {}
        if (len(action) != 0): 
            action = np.squeeze(action)
            vehicle_ids = set(traci.vehicle.getIDList()) #ids are 0-9
            # print("")
            # print("vehicle_ids current: ", vehicle_ids)
            
            arrived_at_destination = traci.simulation.getArrivedIDList()

            # store edge vehicle counts in connection_info.edge_vehicle_count
            # self.get_edge_vehicle_counts()

            local_targets = {}
            # iterate through vehicles currently in simulation
            # print("all vehicles in one step")
            # print(vehicle_ids)
            i = 0
            for vehicle_id in vehicle_ids:
                # print("rewards ", rewards[vehicle_id])
                if vehicle_id not in self.vehicle_IDs_in_simulation and vehicle_id in self.controlled_vehicles:
                    self.vehicle_IDs_in_simulation.append(vehicle_id)
                    traci.vehicle.setColor(vehicle_id, (255, 0, 0)) # set color so we can visually track controlled vehicles
                    self.controlled_vehicles[vehicle_id].start_time = float(self.step)#Use the detected release time as start time
                

                if vehicle_id in self.controlled_vehicles.keys():
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                    self.current_edges[str(vehicle_id)] = current_edge
                    positions[str(vehicle_id)] = traci.vehicle.getPosition(vehicle_id)
                    #adding the state of the vehicle
                    # print("current_edge: " + current_edge)

                    if current_edge not in self.connection_info.edge_index_dict.keys():
                        continue
                    elif current_edge == self.controlled_vehicles[vehicle_id].destination:
                        continue
                    #Distributing Awards
            
                    ## MAKING THE DECISION LIST FOR EACH VEHICLE
                    decision_list = []
                    # print("{} now on: {}, records on {}; {} ".format(vehicle_id, current_edge, self.controlled_vehicles[vehicle_id].current_edge, current_edge!=self.controlled_vehicles[vehicle_id].current_edge))
                    if current_edge != self.controlled_vehicles[vehicle_id].current_edge:
                        self.controlled_vehicles[vehicle_id].current_edge = current_edge
                        self.controlled_vehicles[vehicle_id].current_speed = traci.vehicle.getSpeed(vehicle_id)
                        wrong_decision = False
                        total_length = 0.0
                        # while total_length < connection_info.edge_length_dict[current_edge]:
                        vehicle_id = int(vehicle_id)
                        # print("ACTION BEFORE MAKING ACTION", action)
                        act = self.direction_choices[np.argmax(action[i])] #a direction - s,L,R

                        i += 1
                        # print("action: " + act)
                        if act not in connection_info.outgoing_edges_dict[current_edge]:
                            print("Impossible turns made for vehicle #" + str(vehicle_id) + " : " + act + " @ " + str(current_edge))
                            rewards[str(vehicle_id)] -= 5000
                            continue
                        
                        # print("Choice for " + str(current_edge) + " is: " + act)
                        target_edge = connection_info.outgoing_edges_dict[current_edge][act]
                        current_edge = target_edge
                        decision_list.append(act)
                        total_length += self.connection_info.edge_length_dict[target_edge]

                        decision_list.append(act)
                        # print(self.controlled_vehicles[str(vehicle_id)])
                        # print("before local targets")
                        # print(RouteController.compute_local_target(self, decision_list=decision_list, vehicle=self.controlled_vehicles[str(vehicle_id)]))
                        local_targets[vehicle_id] = RouteController.compute_local_target(self, decision_list=decision_list, vehicle=self.controlled_vehicles[str(vehicle_id)])

            for vehicle_id, local_target_edge in local_targets.items():
                if str(vehicle_id) in traci.vehicle.getIDList():
                    print("Changing the target of {} to {} with length {}".format(vehicle_id, local_target_edge, self.connection_info.edge_length_dict[local_target_edge]))
                    traci.vehicle.changeTarget(vehicle_id, local_target_edge)
                    # print("destination: " + str(self.controlled_vehicles[vehicle_id].destination))
                    self.controlled_vehicles[str(vehicle_id)].local_destination = local_target_edge

            ## ARRIVED SECTION ## 
            for vehicle_id in arrived_at_destination:
                
                if vehicle_id in self.controlled_vehicles:
                    #print the raw result out to the terminal
                    time_span = self.step - int(float(self.controlled_vehicles[vehicle_id].start_time))
                    self.total_time += time_span
                    miss = False
                    arrived = False
                    if self.controlled_vehicles[vehicle_id].local_destination == self.controlled_vehicles[vehicle_id].destination:
                        arrived = True   
                    if self.step > self.controlled_vehicles[vehicle_id].deadline:
                        self.deadlines_missed.append(vehicle_id)
                        miss = True
                    # end_number += 1
                    print("Vehicle {} reaches the destination: {}, timespan: {}, deadline missed: {}"\
                        .format(vehicle_id, arrived, time_span, miss))
                    self.vehicle_IDs_in_simulation.remove(vehicle_id)
        traci.simulationStep()
        self.step += 1
        
        next_vehicle_ids = set(traci.vehicle.getIDList())
        # states = np.zeros(len(vehicle_ids), (len(self.connection_info.edge_list) + 9), dtype=np.float32 ) #only getting states that are currently in the simulation
        states = {}
        arrived_at_destination = traci.simulation.getArrivedIDList()
        arrived_list = list(arrived_at_destination)
        # print(" ")
        # print(next_vehicle_ids)
        for vehicle_id in next_vehicle_ids:
            
            ind_reward = 0
            if vehicle_id in self.controlled_vehicles.keys():
                current_edge = traci.vehicle.getRoadID(vehicle_id)
                #grabbing states of vehicles in the simulation in next step
                state_temp = self.get_state(vehicle_id, current_edge, self.controlled_vehicles[vehicle_id].destination, connection_info.net_filename, positions)

                states[(vehicle_id)] = (state_temp) 
            
            net = sumolib.net.readNet(connection_info.net_filename)
            dest_coord = net.getEdge(self.controlled_vehicles[vehicle_id].destination).getShape()
            dest_coord = dest_coord[len(dest_coord) - 1]
            distance = self.calc_distance(traci.vehicle.getPosition(vehicle_id), dest_coord)
            
            if (current_edge) not in self.connection_info.edge_index_dict.keys():
                continue
            elif current_edge == self.controlled_vehicles[vehicle_id].destination:
                print("BNROO")
                print(current_edge)
                print(self.controlled_vehicles[vehicle_id].destination)
                ind_reward += distance * 0.6
                # arrived_list.append(vehicle_id)
            #Distributing rewards
            # if (traci.vehicle.getAccumulatedWaitingTime(vehicle_id) > 30):
            #     rewards[int(vehicle_id)] -= traci.vehicle.getAccumulatedWaitingTime(vehicle_id) * 0.6
            # elif traci.vehicle.getAccumulatedWaitingTime(vehicle_id) > 10:
            #     rewards[int(vehicle_id)] -= traci.vehicle.getAccumulatedWaitingTime(vehicle_id) * 0.3
            # print(vehicle_id)
            ts = self.step - int(float(self.controlled_vehicles[vehicle_id].start_time))
            if vehicle_id not in arrived_at_destination :
                if (ts > 200):
                    ind_reward -= (ts - 200) * 0.8
                elif (ts > 150):
                    ind_reward -= (ts - 150) * 0.4
            
            if distance < 50:
                ind_reward += (250-distance) * 0.15
            elif (distance < 100):
                ind_reward += (250 - distance) * 0.1
            elif distance < 250:
                ind_reward += (250 - distance) * 0.07
            else:
                ind_reward += (250 - distance) * 0.05
            # print(vehicle_id)
            # print("rewards ", rewards[vehicle_id])
            # print("ind_reward: ", ind_reward)
            # rewards[vehicle_id] += ind_reward
            # print("rewards after", rewards[vehicle_id])
        # print("arrived_at_destination", arrived_at_destination)
        # print("arrived_list", arrived_list)
        for vehicle_id in arrived_list:
            if vehicle_id in self.controlled_vehicles:
                # if vehicle_id not in rewards:
                #     rewards[vehicle_id] = 0
                current_edge = self.current_edges[vehicle_id]
                #grabbing states of vehicles in the simulation in next step
                state_temp = self.get_state(vehicle_id, current_edge, self.controlled_vehicles[vehicle_id].destination, connection_info.net_filename, positions)
                states[(vehicle_id)] = (state_temp) 
            
                # state_temp = self.get_state(vehicle_id, current_edge, self.controlled_vehicles[vehicle_id].destination, connection_info.net_filename)
                #print the raw result out to the terminal
                time_span = self.step - self.controlled_vehicles[vehicle_id].start_time
                arrived = False
                if self.controlled_vehicles[vehicle_id].local_destination == self.controlled_vehicles[vehicle_id].destination:
                    arrived = True   
                    rewards[vehicle_id] += 10000 
                if not arrived: 
                    if (time_span < 40):
                        rewards[vehicle_id] -= 5000 /(time_span * 0.3)
                    else:
                        rewards[vehicle_id] -=  5000 /(time_span * 0.1)
                self.total_time += time_span
                miss = False
                if self.step > self.controlled_vehicles[vehicle_id].deadline:
                    self.deadlines_missed.append(vehicle_id)
                    miss = True
                # end_number += 1
                if traci.simulation.getMinExpectedNumber() == 0:
                    print("Vehicle {} reaches the destination: {}, timespan: {}, deadline missed: {}"\
                        .format(vehicle_id, arrived, time_span, miss))


        if traci.simulation.getMinExpectedNumber() == 0:
            done = True
        # print("printing states before finish with one step", states)
        return states, rewards, done, arrived_list
        # return next_state, reward, done, rewards 

    def reset(self):
        dom = parse("./configurations/myconfig.sumocfg")
        route_file_node = dom.getElementsByTagName('route-files')
        route_file_attr = route_file_node[0].attributes
        route_file = "./configurations/"+route_file_attr['value'].nodeValue
        self.vehicle_IDs_in_simulation = []
        self.deadlines_missed = []
        self.total_time = 0
        self.step = 0
        self.current_edges = {}

        self.controlled_vehicles =  get_controlled_vehicles(route_file, self.connection_info, self.number_vehicles, 50)
        traci.start([self.sumo_binary, "-c", "./configurations/myconfig.sumocfg"], label=self.label)
        # state = np.zeros((self.number_vehicles, (len(self.connection_info.edge_list) + 9)), dtype=np.float32)
        # print(state)
        return {}

    def get_state(self, vid, edge_now, destination, net_file, positions):
        net = sumolib.net.readNet(net_file)
        en = edge_now
        if not net.hasEdge(edge_now):
            en =  self.controlled_vehicles[vid].local_destination
            # print(edge_now + " changed to : " + en)
        dest_edge = net.getEdge(destination)
        end_destination_pos = dest_edge.getShape()
        end_destination_pos = end_destination_pos[len(end_destination_pos) - 1] # (x, y)
        try:
            position = positions[vid] 
        except:
            position = traci.vehicle.getPosition(vid)
        state = []

        state.append(self.connection_info.edge_index_dict[en]) # current edge index
        state.append(self.connection_info.edge_index_dict[destination]) # final location
        state.append(self.calc_distance(position, end_destination_pos)) # distance between current edge and final destination
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

        # state = np.reshape(state, [1, len(state)]) #2D array with 1 row and as many columns as state
        #state = [[edge index, 1, 0 ,0 ,1, 1, 1, density, ... (density of every other edge)]
        return state


    # Calculate traffic density for a specific road segment
    def calculate_traffic_density(self, segment_id):
        vehicle_ids = traci.edge.getLastStepVehicleIDs(segment_id)
        num_vehicles = len(vehicle_ids)
        segment_length = self.connection_info.edge_length_dict[segment_id]
        density = num_vehicles / segment_length
        return density
    
    def calc_distance(self, pos1, pos2):
        return math.sqrt(pow(abs(pos1[0] - pos2[0]), 2) + pow(abs(pos1[1] - pos2[1]), 2))

    def close(self):
        traci.close()
    