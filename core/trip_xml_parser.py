from xml.dom import minidom
import pandas as pd
from core.Util import Vehicle

class Vehicle_info:
    def __init__(self, Round_name):
        """
        Args:
                vehicle_id:         type: string. The id of the vehicle.
                destination:        type: string. The id of the edge where the vehicle targets.
                start_time:         type: float. The step # when the vehicle is released. This value will be updated by STR_SUMO.
                deadline:           type: float. The deadline for this vehicle to reach the end of the target edge.
        """
        self.vehicle_ids = []
        self.destinations = {}
        self.origins = {}
        self.start_times = {}
        self.deadlines = {} # ideally a replacement for deadline_dict
        self.vehicle_list = []
        #static_trip.rou.xml
        #SUMO_Trip_Deadline_Data.csv
        deadlines_csv = pd.read_csv("./configurations/Rounds/"+Round_name+'/SUMO_Trip_Deadline_Data.csv')
        vid = deadlines_csv["vehicle_id"].values
        DL=deadlines_csv["deadline"].values

        #VR_stats = {} # vehicle route stats of form {id:route_info(Object)} route_info is an object in util.py

        for x in range(len(vid)):
            self.deadlines[vid[x]]=DL[x]
            # print (vid[x])
            # print (type(vid[x]))

        xml_file = minidom.parse("./configurations/Rounds/"+Round_name+'/static_trip.rou.xml')
        trips_file = minidom.parse("./configurations/Rounds/"+Round_name+"/trips.trips.xml") 
        trips_info = trips_file.getElementsByTagName('tripinfo')
        vehicle_alts = xml_file.getElementsByTagName('trip')
        for x in trips_info:
            vid = x.getAttribute("id")
            self.vehicle_ids.append(vid)
            vid = int(vid)
            self.destinations[vid] = (x.getAttribute("arrivalLane"))[:-2]
            self.start_times[vid] = x.getAttribute("depart")
            self.origins[vid] = (x.getAttribute("departLane"))[:-2]
            # print(vid)
            # print(self.deadlines[int(vid)])
            self.vehicle_list.append(Vehicle(str(vid), self.destinations[vid], self.start_times[vid], self.deadlines[vid]))

        