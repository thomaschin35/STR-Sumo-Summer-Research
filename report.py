
import csv
from xml.dom import minidom


def report(location, Round_name,iter,run_id,version,deadline_dict={},time=0):
        #function used to record and write data to csv file 
        
        doc2 = minidom.parse(location+'/summary.xml')
        try:# will need 3 output files the summary file, stats_output (traffic incidents) and the trips file
            doc2 = minidom.parse(location+'/summary.xml')
        except Exception as e:
            #print("exception 2:",e)
            try: 
                doc2 = minidom.parse(location+'/summary_%03i.xml'%iter)
            except Exception as e:
                print("exception 2:",e)
                pass
            pass

        step_info = doc2.getElementsByTagName("step")
        last_entry = step_info[len(step_info)-1]
        #print(last_entry)
        total_timespan = last_entry.getAttribute("time") #getting the time of the last entry to the time it took to run.
        #print(total_timespan)
        vehicles_finished = float(last_entry.getAttribute("arrived")) #number of vehicles to finish.
        doc = minidom
        try:
            doc = minidom.parse(location+'/trips.trips.xml')
        except Exception:
            try: 
                doc = minidom.parse(location+'/trips_%03i.trips.xml'%iter)
            except Exception:
                try: 
                    doc = minidom.parse(location+'/tripinfo_%03i.xml'%iter)
                except Exception:
                    pass
        pass

        trip_infos = doc.getElementsByTagName("tripinfo")

        try:
            doc = minidom.parse(location+'/stats_output.xml')
        except Exception:
            pass

        Teleports = doc.getElementsByTagName("teleports")
        Safety = doc.getElementsByTagName("safety")
        #parsing information from output files
        total_travel_time = 0
        max_travel_time = 0
        for x in trip_infos:
            veh_id= float(x.getAttribute("id"))
            veh_fin = float(x.getAttribute("arrival"))
            tt =  float(x.getAttribute("duration"))
            total_travel_time += tt
            if tt > max_travel_time:
                max_travel_time = tt

        round_name_temp = Round_name
        if iter>-1:    
            print(type(round_name_temp))
            round_name_temp = str(round_name_temp) + '-Iteration-'+str(iter)
        # aggregating all information for output to the results.csv file
        average_travel_time = total_travel_time/len(trip_infos)
        temp = []
        overall = []
        temp.append(round_name_temp)
        temp.append(run_id)
        temp.append(version)
        temp.append(total_timespan)
        temp.append(total_travel_time)
        temp.append(average_travel_time)
        temp.append(max_travel_time)
        temp.append(len(trip_infos))
        temp.append(vehicles_finished)
        temp.append(time)
        temp.append(Teleports[0].getAttribute("jam"))
        temp.append(Teleports[0].getAttribute("yield"))
        temp.append(Teleports[0].getAttribute("wrongLane"))
        temp.append(Safety[0].getAttribute("collisions"))
        temp.append(Safety[0].getAttribute("emergencyStops"))
        overall.append(temp)
        csv2Data('./History/results.csv') # getting data from results.csv
        data2Csv_general(overall,'./History/results.csv') #writing data to results.csv

def data2Csv_general(data_selected,directory): 
    with open(directory, 'w',newline='') as f:
        global csv_data
        # using csv.writer method from CSV package
        # literally just used to write data I get from report to results.csv
        write = csv.writer(f)
        pushing = []
        if not csv_data:#csv_data global value I should get from a csv2data method
            pushing = data_selected
        else:
            pushing = csv_data+data_selected
        write.writerows(pushing)    
        f.flush()
        f.close    

def csv2Data(file_name_and_directory):
    global csv_data
    #reading csv data from file to be added onto in report and the data2Csv method
    csv_data = []
    with open(file_name_and_directory,'r',encoding='cp932', errors='ignore') as f2:
        reader = csv.reader(f2)
        for item in reader:
            csv_data.append(item)

if __name__ == "__main__":
    report("./configurations", 1, 1, "run_id_1", "official")