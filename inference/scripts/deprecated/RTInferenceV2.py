#!/usr/bin/env python3

from __future__ import print_function

import warnings
warnings.filterwarnings("ignore")

import os
import cv2
import time
import json
import torch
import numpy as np
import matplotlib
import colorful as cf
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm
import torchvision.models as models

import rospy

from std_msgs.msg import Float32
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from inference.srv import RTInferenceService
from inference.srv import RTInferenceServiceShow
from inferrence.msg import FloatArray

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

device = torch.device("cpu")

############ GLOBAL PARAMS ############
global runid
runid = '02-02-2024_00-45-55'

os.chdir('..')
print(os.getcwd())

class RTinference:
    def __init__(self):
        print('init...')

        ########## MODEL LOAD ##########
        self.load_model()

        ########## PARAMS LOAD ##########
        result = self.read_params_from_json(query_id=runid)
        if result is not None:
            print('params.json query sucessful.')
            self.mean = [result['mean0'], result['mean1'], result['mean2'], result['mean3']]
            self.std = [result['std0'], result['std1'], result['std2'], result['std3']]
        else:
            print("No data found for the specified id.")
            self.mean = None
            self.std = None

        ########## PLOT ##########
        self.fig, _ = plt.subplots(figsize=(8, 5), frameon=True)
        self.image = np.zeros((224, 224))  # empty blank (224, 224) self.image
        self.response = [0.0, 0.0, 0.0, 0.0] # m1, m2, b1, b2

        ############### RUN ###############
        # Set up the ROS subscriber
        self.data = None
        rospy.init_node('RTinference', anonymous=True)
        rospy.Subscriber('/terrasentia/scan', LaserScan, self.lidar_callback)
        rospy.loginfo(cf.green("Server is ready to receive requests"))

        self.pub_mb = rospy.Publisher('/lidar_inference', FloatArray, queue_size=10)
        self.pub_img = rospy.Publisher('/lidar_plot', Image, queue_size=10)
        self.bridge = CvBridge()

        # Set up the ROS service server
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.run()
            except Exception as e:
                print(e)
                pass
            
            rate.sleep()

    ############### ROS INTEGRATION ###############
    def lidar_callback(self, data):
        self.data = data

    def run(self):
        self.generate_image(self.data)
        self.image = self.get_image()

        self.response = self.inference(self.image)

        image_np = np.squeeze(self.image.detach().cpu().numpy())
        ros_image = self.bridge.cv2_to_imgmsg(image_np*250, encoding="passthrough")
        self.pub_img.publish(ros_image)
        self.pub_mb.publish(self.response)
        

    ############### MODEL LOAD ############### 
    def load_model(self):
        ########### MOBILE NET ########### 
        self.model = models.mobilenet_v2()
        self.model.features[0][0] = torch.nn.Conv2d(1, 32, kernel_size=3, stride=2, padding=1, bias=False)

        # MobileNetV2 uses a different attribute for the classifier
        num_ftrs = self.model.classifier[1].in_features
        self.model.classifier[1] = torch.nn.Sequential(
        torch.nn.Linear(num_ftrs, 512),
        torch.nn.BatchNorm1d(512),
        torch.nn.ReLU(inplace=True),
        torch.nn.Linear(512, 3)
        )

        path = os.getcwd() + '/models/' + 'model_' + runid + '.pth'
        checkpoint = torch.load(path, map_location='cpu')  # Load to CPU
        self.model.load_state_dict(checkpoint)
        self.model.eval()

    ############### DATA EXTRACTION ###############

    def read_params_from_json(self, filename='./models/params.json', query_id=None):
        if os.getcwd() == 'scripts':
            os.chdir('..')
        try:
            with open(filename, 'r') as file:
                data = json.load(file)

                if query_id is not None and data['id'] != query_id:
                    return None  # Return None if the queried id does not match

                result_dict = {
                    'id': data['id'],
                    'mean0': data['mean0'],
                    'mean1': data['mean1'],
                    'mean2': data['mean2'],
                    'mean3': data['mean3'],
                    'std0': data['std0'],
                    'std1': data['std1'],
                    'std2': data['std2'],
                    'std3': data['std3']
                }

                return result_dict

        except (FileNotFoundError, json.decoder.JSONDecodeError, KeyError):
            return None

    def generate_image(self, data):
        lidar = data.ranges
        
        min_angle = np.deg2rad(0)
        max_angle = np.deg2rad(180) # lidar range
        angle = np.linspace(min_angle, max_angle, len(lidar), endpoint = False)

        # convert polar to cartesian:
        # x = r * cos(theta)
        # y = r * sin(theta)
        # where r is the distance from the lidar (x in lidar)
        # and angle is the step between the angles measure in each distance (angle(lidar.index(x))
        xl = [x*np.cos(angle[lidar.index(x)]) for x in lidar]
        yl = [y*np.sin(angle[lidar.index(y)]) for y in lidar]

        # take all the "inf" off
        xl = [10.0 if value == 'inf' else value for value in xl]
        yl = [10.0 if value == 'inf' else value for value in yl] 

        POINT_WIDTH = 18
        if len(xl) > 0:
            plt.cla()
            plt.plot(xl,yl, '.', markersize=POINT_WIDTH, color='black')
            plt.axis('off')
            plt.xlim([-1.5, 1.5])
            plt.ylim([0, 2.2])
            plt.grid(False)
            
            plt.gca().spines['top'].set_visible(False)
            plt.gca().spines['right'].set_visible(False)
            plt.gca().spines['bottom'].set_visible(False)
            plt.gca().spines['left'].set_visible(False)
            
            plt.tight_layout()
            plt.gcf().set_size_inches(5.07, 5.07)
            plt.gcf().canvas.draw()

            plt.savefig('temp_image')


    def get_image(self):
        image = cv2.imread("temp_image.png")

        os.remove("temp_image.png")

        # convert image to numpy 
        image = np.array(image)

        # crop image to 224x224 in the pivot point (112 to each side)
        # image = image[100:400, :, :]
        image = image[:,:, 1]
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_LINEAR)

        # add one more layer to image: [1, 1, 224, 224] as batch size
        image = np.expand_dims(image, axis=0)
        image = np.expand_dims(image, axis=0)

        # convert to torch
        image = torch.from_numpy(image).float()
        return image

    ############### INFERENCE AND PLOT ##############

    def inference(self, image):
        # Inicie a contagem de tempo antes da inferência
        start_time = time.time()

        # get the model predictions
        predictions = self.model(image)

        # Encerre a contagem de tempo após a inferência
        end_time = time.time()

        #print('Inference time: {:.4f} ms'.format((end_time - start_time)*1000))
        
        # correct different format inputs
        predictions = predictions.to('cpu').cpu().detach().numpy().tolist()[0]
        if len(predictions) == 3:
            w1, q1, q2 = predictions
            w2 = w1
        elif len(predictions) == 4:
            w1, w2, q1, q2 = predictions
        else: 
            w1, w2, q1, q2 = [None, None, None, None]

        # deprocess
        if not any(e is None for e in [w1, w2, q1, q2]): # enter only if there is no None in the list
            m1, m2, b1, b2 = self.deprocess(label=[w1, w2, q1, q2])
        else:
            m1, m2, b1, b2 = [w1, w2, q1, q2] # Can't use this data

        return [m1, m2, b1, b2]

    def deprocess(self, label):
        ''' Returns the deprocessed image and label. '''

        if len(label) == 3:
            # we suppose m1 = m2, so we can use the same deprocess
            w1, q1, q2 = label
            w2 = w1
        elif len(label) == 4:
            w1, w2, q1, q2 = label

        # DEPROCESS THE LABEL
        w1 = (w1 * self.std[0]) + self.mean[0]
        w2 = (w2 * self.std[1]) + self.mean[1]
        q1 = (q1 * self.std[2]) + self.mean[2]
        q2 = (q2 * self.std[3]) + self.mean[3]

        m1 = 1/w1
        m2 = 1/w2
        b1 = -q1/w1
        b2 = -q2/w2

        return [m1, m2, b1, b2]

############### MAIN ###############

if __name__ == '__main__':
    run = RTinference()
    rospy.spin()