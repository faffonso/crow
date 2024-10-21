#!/usr/bin/env python3
import os
import cv2
import json
import torch
import numpy as np
import colorful as cf
import matplotlib.pyplot as plt
import torchvision.models as models

import rospy
from sensor_msgs.msg import LaserScan, Image
from wp_gen.msg import CropLine
from cv_bridge import CvBridge

from std_srvs.srv import Trigger

class RTInference:
    def __init__(self):
        rospy.init_node('RTinference_node')

        # Load parameters
        self.model_path = rospy.get_param('~model_path', './models')
        self.run_id = rospy.get_param('~run_id', '06-04-2024_21-22-37')
        self.scan_topic = rospy.get_param('~scan_topic', '/terrasentia/scan')
        self.lidar_plot_topic = rospy.get_param('~lidar_plot_topic', '/terrasentia/lidar_plot')
        self.crop_lines_topic = rospy.get_param('~crop_lines_topic', '/terrasentia/crop_lines')
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        rospy.loginfo(f'RTInferenceNode using device: {self.device}')

        # Load the model and parameters
        self.model = self.load_model()
        self.mean, self.std = self.load_params(self.run_id)

        # Initialize plot and CV bridge
        self.fig, _ = plt.subplots(figsize=(8, 5), frameon=True)
        self.image = np.zeros((224, 224))
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.pub_image = rospy.Publisher(self.lidar_plot_topic, Image, queue_size=1)
        self.pub_lines = rospy.Publisher(self.crop_lines_topic, CropLine, queue_size=1)

        self.data = None
        self.response = [0.0, 0.0, 0.0, 0.0] 
        self.lidar_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.lidar_callback, buff_size=2**28)

        # Service to start and stop the recording
        rospy.wait_for_service('/data_recording/start_recording')
        self.start_recording = rospy.ServiceProxy('/data_recording/start_recording', Trigger)

        self.start_recording()

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                self.run()
            except Exception as e:
                print(e)
                pass
            rate.sleep()

    def lidar_callback(self, data):
        '''
        Callback for the lidar data.

        Args:
            data: The lidar data.
        '''
        self.data = data

    def run(self):
        '''
        Run the inference.
        '''
        crop_msg = CropLine()

        if self.data is not None: 
            start_time = rospy.Time.now() 

            self.generate_image(self.data)
            self.image, raw_image = self.get_image()

            crop_lines = self.inference(self.image)
            crop_msg.m1 = crop_lines[0]
            crop_msg.m2 = crop_lines[1]
            crop_msg.b1 = crop_lines[2]
            crop_msg.b2 = crop_lines[3]

            ros_image = self.plot(crop_lines, raw_image)

            end_time = rospy.Time.now() 
            elapsed_time = (end_time - start_time).to_sec() * 1000

            if (elapsed_time > 500):
                print(cf.red(f"Max time ({elapsed_time})"))
                self.pub.publish(ros_image)
                return

            self.pub_image.publish(ros_image)
            self.pub_lines.publish(crop_msg)

    def rt_inference_service(self, req):
        '''
        Service to perform the inference in real-time.

        Args:
            req: The request with the image.

        Returns:
            line1: The first line.
            line2: The second line.
            image: The image.
        '''
        rospy.loginfo(cf.yellow(f"Received request {req}"))
        
        m1, m2, b1, b2 = self.response
        print(f'm1={m1:.2f}, m2={m2:.2f}, b1={b1:.2f}, b2={b2:.2f}')

        line1 = CropLine(m1, b1)
        line2 = CropLine(m2, b2)
        
        if req.show:
            image = self.image.flatten().tolist()
            return line1, line2, image
        else:
            image = []
            return line1, line2, image

    def load_model(self):
        '''
        Load the model from the path.

        Returns:
            model: Neural network model.
        '''
        model = models.mobilenet_v2()
        model.features[0][0] = torch.nn.Conv2d(1, 32, kernel_size=3, stride=2, padding=1, bias=False)

        num_ftrs = model.classifier[1].in_features
        model.classifier[1] = torch.nn.Sequential(
            torch.nn.Linear(num_ftrs, 512),
            torch.nn.BatchNorm1d(512),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(512, 3)
        )

        model_path = os.path.join(self.model_path, f'model_{self.run_id}.pth')
        checkpoint = torch.load(model_path, map_location='cpu', weights_only=True)
        model.load_state_dict(checkpoint)
        model.eval()

        return model

    def load_params(self, run_id):
        '''
        Load the parameters from the JSON file.

        Args:
            run_id: The run ID of the model.

        Returns:
            [mean0, mean1, mean2, mean3]: The mean values of the parameters.
            [std0, std1, std2, std3]: The standard deviation values of the parameters.
        '''
        params_path = os.path.join(self.model_path, 'params.json')
        with open(params_path, 'r') as file:
            data = json.load(file)
            for item in data:
                if item['id'] == run_id:
                    return [item[f'mean{i}'] for i in range(4)], [item[f'std{i}'] for i in range(4)]

        rospy.logwarn(f"No parameters found for run ID {run_id}")
        rospy.signal_shutdown("Parameters not found.")
        return None, None

    def generate_image(self, data):
        '''
        Generate the image from the lidar data.

        Args:
            data: The lidar data.
        '''
        lidar = data.ranges
        min_angle = data.angle_min
        max_angle = data.angle_max 
        angle = np.linspace(min_angle, max_angle, len(lidar), endpoint = False)

        # Convert polar to cartesian coordinates
        yl = [x*np.cos(angle[lidar.index(x)]) for x in lidar]
        xl = [-y*np.sin(angle[lidar.index(y)]) for y in lidar]

        # Replace inf values with 10.0
        xl = [10.0 if value == 'inf' else value for value in xl]
        yl = [10.0 if value == 'inf' else value for value in yl] 

        # Plot the lidar data
        if len(xl) > 0:
            plt.cla()
            plt.plot(xl,yl, '.', markersize=18, color='black')
            plt.axis('off')
            plt.xlim([-1.5, 1.5])
            plt.ylim([0.0, 3.0])
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
        '''
        Get the image from the plot and preprocess it.

        Returns:
            image: The preprocessed image.
            raw_image: The original image.  
        '''
        image = cv2.imread("temp_image.png")
        os.remove("temp_image.png")

        image = np.array(image)
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_LINEAR)
        raw_image = image
        image = image[:,:, 1]

        # Add batch and channel dimensions
        image = np.expand_dims(image, axis=0)  # Channel
        image = np.expand_dims(image, axis=0)  # Batch size

        # Convert to torch tensor and move to the correct device
        image = torch.from_numpy(image).float()
        return image, raw_image

    def inference(self, image):
        '''
        Perform the inference on the image.

        Args:
            image: The image to perform the inference.

        Returns:
            [m1, m2, b1, b2]: The slope and intercept of the lines.
        '''
        predictions = self.model(image)

        predictions = predictions.to('cpu').cpu().detach().numpy().tolist()[0]
        if len(predictions) == 3:
            w1, q1, q2 = predictions
            w2 = w1
        elif len(predictions) == 4:
            w1, w2, q1, q2 = predictions
        else: 
            w1, w2, q1, q2 = [None, None, None, None]

        if not any(e is None for e in [w1, w2, q1, q2]): # Enter only if there is no None in the list
            m1, m2, b1, b2 = self.deprocess(label=[w1, w2, q1, q2])
        else:
            m1, m2, b1, b2 = [w1, w2, q1, q2] # Can't use this data

        return [m1, m2, b1, b2]

    def deprocess(self, label):
        '''
        Deprocess the label to get the slope and intercept of the lines.

        Args:
            label: The label from the inference.

        Returns:    
            [m1, m2, b1, b2]: The slope and intercept of the lines.
        '''
        if len(label) == 3:
            # We suppose m1 = m2, so we can use the same deprocess
            w1, q1, q2 = label
            w2 = w1
        elif len(label) == 4:
            w1, w2, q1, q2 = label

        w1 = (w1 * self.std[0]) + self.mean[0]
        w2 = (w2 * self.std[1]) + self.mean[1]
        q1 = (q1 * self.std[2]) + self.mean[2]
        q2 = (q2 * self.std[3]) + self.mean[3]

        m1 = 1/w1
        m2 = 1/w2
        b1 = -q1/w1
        b2 = -q2/w2

        return [m1, m2, b1, b2]

    def plot(self, response, raw_image):
        '''
        Plot the lines on the image.

        Args:
            response: The response from the inference.
            raw_image: The original image.

        Returns:
            ros_image: The image with the lines plotted.
        '''
        m1, m2, b1, b2 = response

        try: 
            # Calculate the endpoints of the line
            x11 = 0
            y11 = int(m1 * x11 + b1)

            x12 = raw_image.shape[1]  # Width of the image
            y12 = int(m1 * x12 + b1)

            # Draw the line on the image
            line_color = (0, 0, 255)  
            line_thickness = 2
            cv2.line(raw_image, (x11, y11), (x12, y12), line_color, line_thickness)

            # Calculate the endpoints of the line
            x21 = 0
            y21 = int(m2 * x21 + b2)

            x22 = raw_image.shape[1]  # Width of the image
            y22 = int(m2 * x22 + b2)
        except Exception as e: 
            print('Error RTInference (float -> int):', e)

        cv2.line(raw_image, (x21, y21), (x22, y22), line_color, line_thickness)

        ros_image = self.bridge.cv2_to_imgmsg(raw_image, encoding="passthrough")
        return ros_image

if __name__ == '__main__':
    run = RTInference()
    rospy.spin()
