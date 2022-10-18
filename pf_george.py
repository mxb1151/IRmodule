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
            # #construct an array of x, y pairs for each point in the point cloud. e.g. [[x1, y1], [x2, y2], [...]]
            # point_cloud_array = [] 
            # for i in self.particlecloud.poses:
            #     point_cloud_array.append([i.position.x, i.position.y ])

            # #Parameters are being set arbitrarily here: however, these could most likely be optimized with automatic best parameter inferral 
            # dbscan = DBSCAN(eps = 8, min_samples = 4).fit(np.array(x)) # fitting the model
            # clusters = dbscan.labels_ # getting the labels

            # #find biggest cluster (most occuring number)
            # biggest_cluster = max(set(point_cloud_array), key = point_cloud_array.count)

            
            sum_head = 0
            x = []
            y = []
            count = 1


            #construct an array of x, y pairs for each point in the point cloud. e.g. [[x1, y1], [x2, y2], [...]]
            point_cloud_array = [] 
            for i in self.particlecloud.poses:
                point_cloud_array.append([i.position.x, i.position.y ])
            #rospy.loginfo(point_cloud_array)
            #Parameters are being set arbitrarily here: however, these could most likely be optimized with automatic best parameter inferral 
            dbscan = DBSCAN(eps = 8, min_samples = 4).fit(np.array(point_cloud_array)) # fitting the model
            clusters = dbscan.labels_ # getting the labels
            #find biggest cluster (most occuring number)
            rospy.loginfo(list(clusters).count(0))


            biggest_cluster = max(set(point_cloud_array), key = point_cloud_array.count)
            
            rospy.loginfo(biggest_cluster)
            
            
            
            # part = most_frequent(list(clusters))
            # for i in range(len(clusters)):
            #     if part is clusters[i] :
            #         x.append(particles[i].position.x)
            #         y.append(particles[i].position.y)
            #         head = getHeading(particles[i].orientation)
            #         count += 1 
            #         if head < 0 :
            #             head = head + math.pi*2 
            #         sum_head += head 
            # #get all points that are part of the biggest cluster (e.g. cluster with most elements)
            # for i in range(0, len(clusters)): 
            #     if clusters[i] == biggest_cluster: 
            #         x.append(point_cloud_array[i][0])
            #         y.append(point_cloud_array[i][1])



            est_pose = Pose()

            #compute the average position for that cluster
            # get avg_position and heading 
            avg_heading = sum_head / count
            if avg_heading > math.pi:
                avg_heading = avg_heading - math.pi*2
            est_pose.position.x = statistics.median(x)
            est_pose.position.y = statistics.median(y)
            est_pose.position.z = 0 
            est_pose.orientation = rotateQuaternion(Quaternion(w=1.0),avg_heading)

            return est_pose
