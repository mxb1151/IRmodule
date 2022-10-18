from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time

import random

import numpy as np # Kim
from random import gauss # Kim
import statistics 


best_particle = []


def noise_adder(pose):
    rot = random.uniform(0,math.pi)
    trans = random.uniform(0,3)
    pose.position.x += trans
    pose.position.y += trans
    pose.orientation = rotateQuaternion(q_orig = (pose.orientation), yaw = rot)


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        
        self.ODOM_TRANSLATION_NOISE = random.normalvariate(0,0.1) # Kim
        self.ODOM_ROTATION_NOISE = random.normalvariate(0,0.1) # Kim
        self.ODOM_DRIFT_NOISE = random.normalvariate(0,0.1) # Kim

        # ----- Sensor model parameters

        sensor_model = self.sensor_model
        

        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        
        number_of_particles = 5000

        Poses = PoseArray()
        # currentTime = rospy.Time.now()

        # Poses.header.frame_id = "map"
        # Poses.header.stamp = currentTime

        angle = math.pi/4
        rot_mat = np.array([[math.cos(angle),math.sin(angle)],[-math.sin(angle),math.cos(angle)]])
        for i in range(number_of_particles):
            particle_pose = Pose()
            # x_noise = random.uniform(-30,30)
            # y_noise = random.uniform(-15,15)
            # origin_xy = np.array([x_noise,y_noise])
            # [x_hat, y_hat ] = np.dot(rot_mat,origin_xy)
            # particle_pose.position.x = x_hat
            # particle_pose.position.y = y_hat
            # particle_pose.position.x = initialpose.pose.pose.position.x + x_hat
            # particle_pose.position.y = initialpose.pose.pose.position.y + y_hat
            particle_pose.position.x = initialpose.pose.pose.position.x + gauss(0,3)
            particle_pose.position.y = initialpose.pose.pose.position.y + gauss(0,3)
            particle_pose.position.z = 0 
            y = random.uniform(-math.pi,math.pi)
            particle_pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation,y)
            Poses.poses.append(particle_pose)

        self.particlecloud = Poses


        return Poses

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """




        # weightList = []
        # updatedCloud = PoseArray()
        # for pose in self.particlecloud:
        #     weight = self.sensor_model.get_weight(pose = pose,scan = scan)
        #     weightList.append(weight)


        # #Resampling based on weights

        # cWeight = [sum(weightList[:i+1]) for i in range(len(weightList))]
        # i = 0
        # u = random.uniform(0,(1/1000))

        # for j in range(len(self.particlecloud)):
        #     while(u>cWeight[i]):
        #         i = i+1
        #     updatedCloud.poses.append(noise_adder(self.particlecloud.poses[i]))
        #     u += 1/1000

        # self.particlecloud = updatedCloud
        

        number_of_particles = 2000
        sum_of_weights = 0
        
        new_particle_cloud = PoseArray()

        weights = []
        wap = []
        
        for i in self.particlecloud.poses: 
            w = self.sensor_model.get_weight(scan,i)
            sum_of_weights += w 
            weights.append(w)
            wap.append((w,i))
        
        heavy = 0
        for i in range(len(wap)):
            if wap[i][0] > heavy:
                heavy = i

        best_particle.append(wap[heavy][1])
      

        number_of_old_particles = len(weights)
        partial_weights = [x/sum_of_weights for x in weights]
        #partial_weights = weights / sum_of_weights 

        ## Resampling Algorithms 

        # Generate cdf 
        # cummulative_df = np.array([partial_weights[0]])
        # for i in np.arange(1,len(weights)):
        #     cummulative_df = np.append(cummulative_df, cummulative_df[i-1]+partial_weights[i])
        
        cummulative_df = [sum(partial_weights[:i+1]) for i in range(len(partial_weights))]

        # initialize threshold 
        u_threshold = number_of_particles**-1
        u_1 = random.uniform(0, number_of_particles**-1)
        k = 0

        # Draw Samples 
        S = []
        # u_j = np.array([u_1])
        # u_j = np.append(u_j,np.zeros(number_of_particles-1))
        # print(u_j[0:10])


        noise1 = 0 
        noise2 = 0 
        for j in range(number_of_particles-1):
            
            while(u_1 > cummulative_df[k]):
                k = k + 1 
            #     noise1 = random.normalvariate(0,10)
            #     noise2 = random.normalvariate(0,10)
            # self.particlecloud.poses[k].position.x = self.particlecloud.poses[k].position.x + noise1
            # self.particlecloud.poses[k].position.y = self.particlecloud.poses[k].position.y + noise2
            
            S.append([self.particlecloud.poses[k],u_1])
            u_1 = u_1 + u_threshold
           
        
        # Generate particles pose with noise 

        for i in S:     
            p = i[0]
            particle_pose = Pose()
            rnd = random.normalvariate(0,0.1)
            rnd1 = random.normalvariate(0,0.1)
            particle_pose.position.x = p.position.x + rnd
            particle_pose.position.y = p.position.y + rnd1 
            particle_pose.position.z = p.position.z
            y = random.uniform(0,math.pi)
            particle_odom = p.orientation
            particle_pose.orientation = rotateQuaternion(particle_odom,y)
            
            new_particle_cloud.poses.append(particle_pose)

        self.particlecloud = new_particle_cloud


    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        ### Demo
        number_of_particles = len(self.particlecloud.poses)
        sum_x = 0 
        sum_y = 0 
        sum_z = 0
        sum_odom_x = 0 
        sum_odom_y = 0 
        sum_odom_z = 0
        sum_odom_w = 0
        sum_yaw = 0 
        x = []
        y = []
        z = []
        for i in self.particlecloud.poses:
            x.append(i.position.x)
            y.append(i.position.y)
            z.append(i.position.z)
            sum_x += i.position.x 
            sum_y += i.position.y 
            sum_z += i.position.z
            sum_yaw = getHeading(i.orientation) 
        est_pose = Pose()
        # est_pose.position.x = sum_x / number_of_particles
        # est_pose.position.y = sum_y / number_of_particles
        # est_pose.position.z = sum_z / number_of_particles
        # est_pose.position.x = statistics.median(x)
        # est_pose.position.y = statistics.median(y)
        # est_pose.position.z = statistics.median(z)
        # avg_heading = sum_yaw / number_of_particles
        # est_pose.orientation = rotateQuaternion(Quaternion(w=1.0),avg_heading)
        # est_pose.orientation.x = sum_odom_x / number_of_particles
        # est_pose.orientation.y = sum_odom_y / number_of_particles
        # est_pose.orientation.z = sum_odom_z / number_of_particles
        # est_pose.orientation.w = sum_odom_w / number_of_particles
        for i in best_particle:
            sum_x += i.position.x
            sum_y += i.position.y
            sum_yaw = getHeading(i.orientation)

        rnd = random.normalvariate(0,0.1)
        rnd1 = random.normalvariate(0,0.1)
        est_pose.position.x = best_particle[-1].position.x
        est_pose.position.y = best_particle[-1].position.y

        rospy.loginfo(best_particle[-1])
        est_pose = best_particle[-1]
        


        return est_pose
        

