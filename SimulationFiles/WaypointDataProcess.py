#A Module for Processing the Waypoint Data
import csv

def waypointfileProcessor(csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader,None)
            for row in reader:
                column_data.append(row)
            for i in range(len(column_data)):
                 column_data[i]=int(column_data[i][0])
        return column_data

data=waypointfileProcessor('waypointIDS.csv')
print(data[0])