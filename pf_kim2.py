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



"""
	Estimate position based on weighted resampling 
"""





bp_mode = 0
bp2_mode = 0
bp2_x = 0
bp2_y = 0
bp_x = 0 
bp_y = 0

def most_frequent(List):
    counter = 0
    num = List[0]
    curr_frequency = 0
    for i in List:
        curr_frequency = List.count(i)
        if(curr_frequency> counter):
            counter = curr_frequency
            num = i
 
    return num

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

        sensor_model.z_hit = 0.95 
        sensor_model.z_short = 0.1 
        sensor_model.z_max = 0.05
        sensor_model.z_rand = 0.05 
        sensor_model.sigma_hit = 0.2
        sensor_model.lambda_short = 0.05
        

        self.NUMBER_PREDICTED_READINGS = 20   # Number of readings to predict
        
       
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
        
        number_of_particles = 2000

        Poses = PoseArray()
        # currentTime = rospy.Time.now()

        # Poses.header.frame_id = "map"
        # Poses.header.stamp = currentTime

        angle = math.pi/4
        rot_mat = np.array([[math.cos(angle),math.sin(angle)],[-math.sin(angle),math.cos(angle)]])
        for i in range(number_of_particles):
            particle_pose = Pose()
            # x_noise = random.uniform(-15,15)
            # y_noise = random.uniform(-9,9)
            # origin_xy = np.array([x_noise,y_noise])
            # [x_hat, y_hat ] = np.dot(rot_mat,origin_xy)
            # particle_pose.position.x = x_hat
            # particle_pose.position.y = y_hat
            # particle_pose.position.x = initialpose.pose.pose.position.x + x_hat
            # particle_pose.position.y = initialpose.pose.pose.position.y + y_hat
            particle_pose.position.x = initialpose.pose.pose.position.x + gauss(0,2)
            particle_pose.position.y = initialpose.pose.pose.position.y + gauss(0,2)
            particle_pose.position.z = 0 
            y = random.uniform(-math.pi,math.pi)
            particle_pose.orientation = rotateQuaternion(Quaternion(w=1.0),y)
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
        

        number_of_particles = 200
        sum_of_weights = 0

        old_particle_cloud = self.particlecloud 
        new_particle_cloud = PoseArray()

        weights = []
        
        # Store weights for each particle 
        for i in old_particle_cloud.poses: 
            w = self.sensor_model.get_weight(scan,i)
            sum_of_weights = sum_of_weights + w 
            weights.append([w])
        

        ## Normalise 
        number_of_old_particles = len(weights)
        weights = np.array(weights)
        partial_weights = weights / sum_of_weights 


        ############################ Resampling Algorithms #################################### 


        # Generate cdf 
        cummulative_df = np.array([partial_weights[0]])
        for i in np.arange(1,len(weights)):
            cummulative_df = np.append(cummulative_df, cummulative_df[i-1]+partial_weights[i])

        # initialize threshold 
        u_threshold = number_of_particles**-1
        u_1 = random.uniform(0, number_of_particles**-1)
        k = 0


        # Store new particle.poses with threshold 
        S = []
        noise1 = 0 
        noise2 = 0 
        best_particle = []
        for j in range(number_of_particles-1):
            
            while(u_1 > cummulative_df[k]):
                k = k + 1 
            #     noise1 = gauss(0,0.05)
            #     noise2 = gauss(0,0.05)
            # old_particle_cloud.poses[k].position.x = old_particle_cloud.poses[k].position.x + noise1
            # old_particle_cloud.poses[k].position.y = old_particle_cloud.poses[k].position.y + noise2
            
            S.append([old_particle_cloud.poses[k],u_1])
            u_1 = u_1 + u_threshold
            best_particle.append(k)



        ###################### find densest cluster ######################################

        # Store the most frequent old particle index (it will use in the third function)
        global bp_mode 
        global bp2_mode
        bp_mode= most_frequent(best_particle)
        
        new_ar = []        
        for i in best_particle:
            if i is not bp_mode:
                new_ar.append(i)

        bp2_mode = most_frequent(new_ar)
        global bp_x 
        global bp_y
        global bp2_x
        global bp2_y
        bp_x = old_particle_cloud.poses[bp_mode].position.x
        bp_y = old_particle_cloud.poses[bp_mode].position.y
        bp2_x = old_particle_cloud.poses[bp2_mode].position.x
        bp2_y = old_particle_cloud.poses[bp2_mode].position.y



        # Generate particles pose with noise
        for i in S:     
            p = i[0]
            particle_pose = Pose()
            rnd = gauss(0,0.2)
            rnd1 = gauss(0,0.2)
            particle_pose.position.x = p.position.x + rnd
            particle_pose.position.y = p.position.y + rnd1 
            particle_pose.position.z = p.position.z
            y = gauss(0,0.3)
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


        number_of_particles = len(self.particlecloud.poses)
        est_pose = Pose()
    
        sum_head = 0
        x = []
        y = []
        count = 1

        # find particles inside of circle centred at the best particle position
        # store position, orientation of particles 
        rad1 = 0.3
        rad2 = 0.2
        for i in self.particlecloud.poses:
            x_position = i.position.x 
            y_position = i.position.y 
            if abs(x_position - bp_x) < rad1 or abs(y_position - bp_y) < rad1 :
                x.append(x_position)
                y.append(y_position)
                head = getHeading(i.orientation)
                count +=1
                if head < 0 :
                    head = head + math.pi*2
                    sum_head = sum_head + head 
            elif abs(x_position - bp2_x) < rad2 or abs(y_position - bp2_y) < rad2 :
                x.append(x_position)
                y.append(y_position)
                head = getHeading(i.orientation)
                count +=1
                if head < 0 :
                    head = head + math.pi*2
                    sum_head = sum_head + head 


        # get avg_position and heading 
        avg_heading = sum_head / (count-1)
        if avg_heading > math.pi:
            avg_heading = avg_heading - math.pi*2
        est_pose.position.x = statistics.median(x)
        est_pose.position.y = statistics.median(y)
        est_pose.position.z = 0 
        est_pose.orientation = rotateQuaternion(Quaternion(w=1.0),avg_heading)

        return est_pose
        

