#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
The script processes a ROS bag file to extract LaserScan data from a specific topic and save it into a CSV file. Here's what it does in a nutshell:

- Preparation:
    - The script imports the necessary modules (rosbag, csv, os, and sensor_msgs.msg.LaserScan).
    - It sets a global variable fid to 5 and changes the working directory to the parent folder.
- Extracting LaserScan Data:
    - The extract_topic_messages function reads through the ROS bag and collects LaserScan messages from the "/terrasentia/scan" topic, storing them as a list of tuples containing the timestamp and scan ranges.
- Writing to CSV:
    - The laserscan_messages_to_csv function writes the extracted LaserScan data to a CSV file.
    - It includes a header row with the timestamp and the number of scan ranges.
    - For each message, it converts the timestamp to seconds and writes the scan data to the CSV file.
- Main Execution:
    - The main block specifies the ROS bag file and output CSV file based on the fid variable.
    - It then uses the defined functions to extract the LaserScan data and save it to the CSV file.

In summary, the script is designed to read a ROS bag file, extract LaserScan messages from a given topic, and save the data into a CSV file for further analysis or processing.
'''

import rosbag
import csv
from sensor_msgs.msg import LaserScan, Image
import os

global fid 
fid = 5

os.chdir('..') 

def extract_topic_messages(bag_filename):
    bag = rosbag.Bag(bag_filename, "r")
    messages_laser = []
    for topic, msg, t in bag.read_messages():
        if topic == "/terrasentia/scan":
            messages_laser.append((t, list(msg.ranges)))
    bag.close()
    return messages_laser

def laserscan_messages_to_csv(messages, output_filename):
    with open(output_filename, "w") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp"] + ["lidar (" + str(len(messages[0][1])) + ")" ])
        # write empty line
        writer.writerow([])
        for t, ranges in messages:
            row = [t.to_sec()] + ranges
            writer.writerow(row)
            writer.writerow([])

if __name__ == "__main__":
    bag_filename = "../datasets/gazebo/crop" + str(fid) + ".bag"
    
    output_filename = "../datasets/Crop_Data" + str(fid) + ".csv"
    messages_laser = extract_topic_messages(bag_filename)

    laserscan_messages_to_csv(messages_laser, output_filename)
