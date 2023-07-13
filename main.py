'''
This test file needs the following files:
STR_SUMO.py, RouteController.py, Util.py, test.net.xml, test.rou.xml, myconfig.sumocfg and corresponding SUMO libraries.
'''
import csv
from xml.dom import minidom
from core.STR_SUMO import StrSumo
import os
import sys
import pandas as pd
import numpy as np
from xml.dom.minidom import parse, parseString
from core.Util import *
from core.trip_xml_parser import Vehicle_info
from controller.RouteController import *
from controller.DijkstraController import DijkstraPolicy
from controller.QLearningController import QLearningPolicy
from controller.TestController import TestPolicy
from core.target_vehicles_generation_protocols import *

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from sumolib import checkBinary
import traci


# use vehicle generation protocols to generate vehicle list
def get_controlled_vehicles(route_filename, connection_info, \
    num_controlled_vehicles=10, num_uncontrolled_vehicles=20, pattern = 3):
    '''
    :param @route_filename <str>: the name of the route file to generate
    :param @connection_info <object>: an object that includes the map inforamtion
    :param @num_controlled_vehicles <int>: the number of vehicles controlled by the route controller
    :param @num_uncontrolled_vehicles <int>: the number of vehicles not controlled by the route controller
    :param @pattern <int>: one of four possible patterns. FORMAT:
            -- CASES BEGIN --
                #1. one start point, one destination for all target vehicles
                #2. ranged start point, one destination for all target vehicles
                #3. ranged start points, ranged destination for all target vehicles
            -- CASES ENDS --
    '''
    vehicle_dict = {}
    print(connection_info.net_filename)
    generator = target_vehicles_generator(connection_info.net_filename)

    # list of target vehicles is returned by generate_vehicles
    # vehicle_list = generator.generate_vehicles(num_controlled_vehicles, num_uncontrolled_vehicles, \
    #     pattern, route_filename, connection_info.net_filename)

    vehicle_list = make_Trips_File(connection_info,"test1", connection_info.net_filename, num_controlled_vehicles, num_uncontrolled_vehicles)
    # generate_Random_Vehicles(num_uncontrolled_vehicles, connection_info.net_filename, route_filename)

    for vehicle in vehicle_list:
        vehicle_dict[str(vehicle.vehicle_id)] = vehicle

    return vehicle_dict

def make_Trips_File(connection_info,Round_name,net_file,num_trips, num_random_vehicles,start_edges=[],end_edges=[],dependencies=[],use_random_trips=True,maximum_release_time=400):
    print("Starting")
    if num_trips == -1:
        return
    cwd = os.getcwd()
    # generator = target_vehicles_generator(connection_info.net_filename)
    if not os.path.isdir("./configurations/Rounds/"+Round_name):
        print("First in")
        os.mkdir("./configurations/Rounds/"+Round_name)
        deadlines= []
        vehicle_list = []
        root = minidom.Document()
        xml = root.createElement('routes')
        xml.setAttribute('xmlns:xsi','"http://www.w3.org/2001/XMLSchema-instance"')
        xml.setAttribute('xsi:noNamespaceSchemaLocation','"http://sumo.dlr.de/xsd/routes_file.xsd"')
        root.appendChild(xml)

        release_times =[]
        for x in range(0,num_trips):
            release_times.append(random.randint(0,maximum_release_time)) # 400
        release_times = sorted(release_times)

        edge_lengths={}
        #Use this for random trips please.
        if use_random_trips or not start_edges:
            net = sumolib.net.readNet(net_file)
            start_edges = []
            end_edges = []
            edges = net.getEdges()
            dependencies = {}
            print("I am in")
            for current_edge in edges:
                current_edge_id = current_edge.getID()
                edge_lengths[current_edge_id]= current_edge.getLength() 
                lanes = current_edge.getLanes()
                
                # "passenger" is a SUMO defined vehicle class
                if current_edge.allows("passenger"):
                    start_edges.append(current_edge_id)
                    end_edges.append(current_edge_id)
                    dependencies[current_edge_id] = [current_edge_id]#to prevent trips to and from the same edge aka very short trips

        
        for x in range(0,num_trips):
            while True:
                start_iterator = random.randint(0,len(start_edges)-1)
                start_edge = start_edges[start_iterator]
                end_iterator = random.randint(0,len(end_edges)-1)
                end_edge = end_edges[end_iterator]
                while edge_lengths[end_edge] <= 10:
                    end_iterator = random.randint(0,len(end_edges)-1)
                    end_edge = end_edges[end_iterator]
                if start_edge in dependencies.keys(): # this should allow for the format of edge dependencies to be much easier.
                    if end_edge not in dependencies[start_edge] and validate_path(net, net.getEdge(start_edge), net.getEdge(end_edge)): #should check if the random route is assigned where the road leads to a dead end 
                        break
            
            # id = x + (num_random_vehicles * 2) + 1
            id = x
            ddl_now = random.randint(300,800)+release_times[x]
            deadlines.append([str(id),ddl_now])
            v_now = Util.Vehicle(str(id), end_edge, release_times[x], ddl_now)
            vehicle_list.append(v_now)
            # child = root.createElement('trip')
            # child.setAttribute('id',str(id))
            # child.setAttribute('depart',str(release_times[x]))
            # child.setAttribute('from',str(start_edge))
            # child.setAttribute('to',str(end_edge))
            child = root.createElement('vehicle')
            child.setAttribute('id',str(id))
            child.setAttribute('depart',str(release_times[x]))
            # child.setAttribute('arrivalPos', "random") - COULD CHANGE LATER TO ADD RANDOM NESS ( USE RAND TO HAVE A CHANCE)
            temp_r = root.createElement('route')
            temp_edges = str(start_edge)
            temp_r.setAttribute('edges', temp_edges)
            child.appendChild(temp_r)
            # child.setAttribute('from',str(start_edge))
            # child.setAttribute('to',str(end_edge))
            # if use_random_trips or not start_edges:
            #     # print(" ")
            #     # print(int(edge_lengths[start_edge]))
            #     # print(int(edge_lengths[end_edge]))
            #     start_len =   random.randint(0,int(edge_lengths[start_edge]))
            #     end_len = random.randint(10,int(edge_lengths[end_edge])) 
            #     child.setAttribute('departPos',str(start_len))
            #     child.setAttribute('arrivalPos',str(end_len))

            xml.appendChild(child)
            
        
        data2Csv_Deadlines(deadlines,Round_name) #insert file name into this as argument
        
        
        xml_str = root.toprettyxml(indent ="\t") 
        print("xml_str_make_trips_type= ",type(xml_str))
        print("xml_str_make_trips= ",xml_str)
        trip_file = "./configurations/Rounds/"+Round_name+'/static_trip.rou.xml'
        with open(trip_file,"w") as f:
            #print(xml_str)
            f.write(xml_str)
            f.flush()
            f.close
        return vehicle_list
    else: 
        vehicle_list = Vehicle_info(Round_name).vehicle_list
        return vehicle_list
        
def generate_Random_Vehicles(num_random_vehicles, netfile, target_xml_file):
    #calculate the density of vehicles accordingly
    latest_release_time = 50.0 #a constant number for the latest release time of all vehicles
    num_random_vehicles *= 2 # this is done to compensate the loss when generating using scripts. Need to solve this later.
    density =  latest_release_time / float(num_random_vehicles)
    density = int(density * 100)/100.0
    #invoke randomTrips.py
    print("net_xml_file:", netfile)
    print("what's our target",target_xml_file)
    command_str = "python randomTrips.py -n "+netfile+" -e 50 -p "+str(density) +" -r "+target_xml_file
    if os.system(command_str) != 0:
        print("ERROR: Failed to invoke randomTrips.py.")
        return None


def data2Csv_Deadlines(deadlines_pushing,Round_name=""):
    with open('./configurations/rounds/'+Round_name+'/SUMO_Trip_Deadline_Data.csv', 'w',newline='') as f:
       
        write = csv.writer(f)
        pushing = []
        Csv_data_static = [["vehicle_id","deadline"]] # this needs to be just the heads of the columns
       
        pushing = Csv_data_static+deadlines_pushing
        write.writerows(pushing)      
        f.close



def test_dijkstra_policy(vehicles):
    print("Testing Dijkstra's Algorithm Route Controller")
    scheduler = DijkstraPolicy(init_connection_info)
    # scheduler = TestPolicy(init_connection_info)
    # scheduler = QLearningPolicy(init_connection_info, "./test/rl-high-all-fixed-late.h5")
    run_simulation(scheduler, vehicles)


def run_simulation(scheduler, vehicles):

    simulation = StrSumo(scheduler, init_connection_info, vehicles)

    traci.start([sumo_binary, "-c", "./configurations/myconfig.sumocfg", \
                 "--tripinfo-output", "./configurations/trips.trips.xml", \
                 "--fcd-output", "./configurations/testTrace.xml","--quit-on-end"])

    total_time, end_number, deadlines_missed = simulation.run()
    print("Average timespan: {}, total vehicle number: {}".format(str(total_time/end_number),\
        str(end_number)))
    print(str(deadlines_missed) + ' deadlines missed.')
    # print(init_connection_info.vehicle_states)
    states2Csv(init_connection_info.vehicle_states, 1)
    # report("./configurations", 1, 1, "run_id_1", "official")

def states2Csv (vehicle_states, Round_name, time=0) :
    generic_header = 'traffic_density'
    specific_headers = ['step', 'vehicle_id', 'edge_ind', 'next_dir', 'straight', 'turn_around', 
                        'slight_right', 'right', 'slight_left', 'left']
    headers= specific_headers + [generic_header]
    temp = []

    for vehicle in vehicle_states:
        arr = vehicle[0]
        temp.append(arr)
    
    with open('./Data/vehicle_states.csv', 'w', newline='') as f:
        write = csv.writer(f)
        write.writerow(headers)
        write.writerows(temp)
        f.flush()


if __name__ == "__main__":
    sumo_binary = checkBinary('sumo-gui')
    # sumo_binary = checkBinary('sumo')#use this line if you do not want the UI of SUMO

    # parse config file for map file name
    dom = parse("./configurations/myconfig.sumocfg")

    net_file_node = dom.getElementsByTagName('net-file')
    net_file_attr = net_file_node[0].attributes

    net_file = net_file_attr['value'].nodeValue
    init_connection_info = ConnectionInfo("./configurations/"+net_file)

    route_file_node = dom.getElementsByTagName('route-files')
    route_file_attr = route_file_node[0].attributes
    route_file = "./configurations/"+route_file_attr['value'].nodeValue
    vehicles = get_controlled_vehicles(route_file, init_connection_info, 150, 50)
    #print the controlled vehicles generated
    for vid, v in vehicles.items():
        print("id: {}, destination: {}, start time:{}, deadline: {};".format(vid, \
            v.destination, v.start_time, v.deadline))
    test_dijkstra_policy(vehicles)
