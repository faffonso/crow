#!/usr/bin/env python

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm

import rospy
from inference.srv import RTInferenceService
from inference.srv import RTInferenceServiceShow

global SHOW
SHOW = True

class clientRTInference:
    def __init__(self):
        rospy.init_node('client_node')
        rospy.wait_for_service('rt_inference_service')
        
        try:
            # Create a service proxy
            if SHOW:
                rt_inference_service = rospy.ServiceProxy('rt_inference_service', RTInferenceServiceShow)
            else:       
                rt_inference_service = rospy.ServiceProxy('rt_inference_service', RTInferenceService)
            
            # Call the service with the request
            response = rt_inference_service()
            
            # Access the response fields
            m1 = response.m1
            m2 = response.m2
            b1 = response.b1
            b2 = response.b2
            if SHOW:
                image = response.image
            else: 
                image = np.zeros(224*224)
        
            rospy.loginfo(f"Client received a response: m1={m1}, m2={m2}, b1={b1}, b2={b2}")
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        x = np.arange(224)
        y1p, y2p, image = self.prepare_plot(x, [m1, m2, b1, b2], image)
        if SHOW:
            self.show(x, y1p, y2p, image)


    def prepare_plot(self, x, predictions, image):
        # line equations explicitly

        # get the slopes and intercepts
        m1p, m2p, b1p, b2p = predictions

        # get the x and y coordinates of the lines
        y1p = m1p*x + b1p
        y2p = m2p*x + b2p

        image = np.array(image)

        # reshape
        image = image.reshape((224, 224))


        return y1p, y2p, image


    def show(self, x, y1p, y2p, image):

        linewidth = 2.5
        fig, ax = plt.subplots(figsize=(8, 5), frameon=True)

        ax.plot(x, y1p, color='red', label='Predicted', linewidth=linewidth)
        ax.plot(x, y2p, color='red', linewidth=linewidth)


        border_style = dict(facecolor='none', edgecolor='black', linewidth=2)
        ax.add_patch(plt.Rectangle((0, 0), 1, 1, **border_style, transform=ax.transAxes))
        plt.legend(loc='upper right', prop={'size': 9, 'family': 'Ubuntu'})
        ax.imshow(image, cmap='magma', norm=PowerNorm(gamma=16), alpha=0.65)
        ax.axis('off')
        plt.show()


if __name__ == "__main__":
    clientRTInference()