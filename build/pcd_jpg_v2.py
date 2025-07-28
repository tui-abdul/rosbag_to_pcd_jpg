import rclpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy import qos
import message_filters
from sensor_msgs_py.point_cloud2 import read_points
import ros2_numpy
import open3d as o3d
import subprocess
import yaml
import os
import shutil
import save_pcd_module
import pickle



class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        with open("../param.yaml","r") as file_handler:
            load_data = yaml.safe_load(file_handler)
        with open(load_data["intrinsic_param_path_1108_603"] , "r") as file_handle:
            self.calib_data_left = yaml.safe_load(file_handle)
            self.matrix_coefficients_left =    self.calib_data_left["camera_matrix"]["data"]
            self.distortion_coefficients_left = self.calib_data_left["distortion_coefficients"]["data"]
            self.matrix_coefficients_left = np.array(self.matrix_coefficients_left).reshape(3,3)
            self.distortion_coefficients_left = np.array(self.distortion_coefficients_left)

        with open(load_data["intrinsic_param_path_1108_618"] , "r") as file_handle:
            self.calib_data_right = yaml.safe_load(file_handle)
            self.matrix_coefficients_right =    self.calib_data_right["camera_matrix"]["data"]
            self.distortion_coefficients_right = self.calib_data_right["distortion_coefficients"]["data"]
            self.matrix_coefficients_right = np.array(self.matrix_coefficients_right).reshape(3,3)
            self.distortion_coefficients_right = np.array(self.distortion_coefficients_right)

        with open(load_data["intrinsic_param_path_0372_302"] , "r") as file_handle:
            self.calib_data_left2 = yaml.safe_load(file_handle)
            self.matrix_coefficients_left2 =    self.calib_data_left2["camera_matrix"]["data"]
            self.distortion_coefficients_left2 = self.calib_data_left2["distortion_coefficients"]["data"]
            self.matrix_coefficients_left2 = np.array(self.matrix_coefficients_left2).reshape(3,3)
            self.distortion_coefficients_left2 = np.array(self.distortion_coefficients_left2)

        with open(load_data["intrinsic_param_path_0372_622"] , "r") as file_handle:
            self.calib_data_right3 = yaml.safe_load(file_handle)
            self.matrix_coefficients_right3 =    self.calib_data_right3["camera_matrix"]["data"]
            self.distortion_coefficients_right3 = self.calib_data_right3["distortion_coefficients"]["data"]
            self.matrix_coefficients_right3 = np.array(self.matrix_coefficients_right3).reshape(3,3)
            self.distortion_coefficients_right3 = np.array(self.distortion_coefficients_right3)
        self.interval_start = int(load_data["interval_start"])
        self.interval_end = int(load_data["interval_end"])
        self.count = 0
        self.count_1 = 0
        folders = ["camera_image_0", "camera_image_1","camera_image_2","camera_image_3", "lidar_point_cloud_0","lidar_point_cloud_1","camera_config", "weather_data", "extraction_info"]
        root_path = load_data["root_path"]
        sequence_num = load_data["sequence_num"]  
        sequence_folder = "sequence_" + str(sequence_num) + "/" 
        root_path = os.path.join(root_path, sequence_folder)
        # Loop to create each folder
        for folder in folders:
            # Create folder if it doesn't already exist
            folder = os.path.join(root_path, folder)
            os.makedirs(folder, exist_ok=True)

        self.get_logger().info('4 folders created successfully!')
        self.folder_image_1 =  root_path + 'camera_image_0/'
        self.folder_image_2 = root_path + 'camera_image_1/'
        self.folder_image_3 = root_path + 'camera_image_2/'
        self.folder_image_4 = root_path + 'camera_image_3/'
        self.folder_pc = root_path + 'lidar_point_cloud_0/'
        self.folder_pc_1 = root_path + 'lidar_point_cloud_1/'
        self.file_1 = open(self.folder_image_1 + 'timestamps.txt', 'w')
        self.file_2 = open(self.folder_image_2 + 'timestamps.txt', 'w')
        self.file_3 = open(self.folder_pc + 'timestamps.txt', 'w')

        self.file_4 = open(self.folder_image_3 + 'timestamps.txt', 'w')
        self.file_5 = open(self.folder_image_4 + 'timestamps.txt', 'w')
        
        self.file_6 = open(self.folder_pc_1 + 'timestamps.txt', 'w')

        bag_file_path = load_data["bag_file_path"]
        command = ["ros2", "bag", "play", bag_file_path, "--rate", "0.1"]# , "--pause"]
        unpause_command = [
                                "ros2", "service", "call",
                                "/pause_playback", "std_srvs/srv/SetBool",
                                "{data: false}"
                            ]


        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("First command started, now starting the next command.")

        
        image_color ='/basler_pole_a_left_id_103_sn_603/my_camera/pylon_ros2_camera_node/image_raw'
        ouster = '/ouster_pole_a_1108/points'
        image_color1 ='/basler_pole_a_right_id_104_sn_618/my_camera/pylon_ros2_camera_node/image_raw'

        image_color2 ='/basler_pole_b_left_id_122_sn_622/my_camera/pylon_ros2_camera_node/image_raw'
        ouster1 = '/ouster_pole_b_0372/points'
        image_color3 ='/basler_pole_b_right_id_121_sn_302/my_camera/pylon_ros2_camera_node/image_raw'

        # Subscribe to topics
        image_sub = message_filters.Subscriber(self, Image, image_color)
        image_sub1 = message_filters.Subscriber(self, Image, image_color1)
        ouster_sub = message_filters.Subscriber(self, PointCloud2, ouster,qos_profile= qos.qos_profile_sensor_data)

        image_sub2 = message_filters.Subscriber(self, Image, image_color2)
        image_sub3 = message_filters.Subscriber(self, Image, image_color3)
        ouster_sub1 = message_filters.Subscriber(self, PointCloud2, ouster1,qos_profile= qos.qos_profile_sensor_data)

                
        #subprocess.run(unpause_command)
        #self.get_logger().info('unpaused the ros2 bag playback')
        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub,image_sub1,image_sub2,image_sub3,ouster_sub,ouster_sub1], queue_size=30, slop=0.075)#, allow_headerless=True)
        ats.registerCallback(self.callback)

        self.bridge = CvBridge()
        



        self.get_logger().info('Initialization complete')

    def transformation_ouster(self,pc_as_numpy_array):
        t_mat =np.array([
                [-1, 0, 0, 0], 
                [0, -1, 0, 0], 
                [0, 0, 1, 0.038195], 
                [0, 0, 0, 1]
            ])
        column_of_ones = np.ones((pc_as_numpy_array.shape[0] , 1))

        result_array = np.hstack((pc_as_numpy_array, column_of_ones))
        transformed_point_3d = np.dot(t_mat, result_array.T)
        #print('t_point',transformed_point_3d.shape)
        return transformed_point_3d.T
    
    def transform_to_ground_plane(self,points):
        rotation_matrix = np.array([
            [ 0.95166884,  0.00776013, -0.30702801],
            
            [ 0.00776013,  0.99875402,  0.04929694],
            
            [ 0.30702801, -0.04929694,  0.95042286]
        ])
        aligned_points = np.dot(points, rotation_matrix)

        return aligned_points
    
    def time_calc_start_end(self,interval_start,interval_end,first_timestamp):
        return first_timestamp + interval_start, first_timestamp + interval_end
    
    
    def callback(self, image_msg,image_msg1,image_msg2,image_msg3,ouster_msg,ouster_msg1):
        self.count = self.count + 1
        filename = str(self.count_1)
        if self.count == 1:
            self.first_timestamp = image_msg.header.stamp.sec
        #self.get_logger().info('New message arrived')
        print("new message arrived",self.count)
        start_timestamp,end_timestamp=self.time_calc_start_end(self.interval_start, self.interval_end, self.first_timestamp)


        
            # Write some content to the file
        self.file_1.write(f"{self.count},{self.count_1}  , {image_msg.header.stamp.sec}.{image_msg.header.stamp.nanosec} \n")
        self.file_2.write(f"{self.count} ,{self.count_1} , {image_msg1.header.stamp.sec}.{image_msg1.header.stamp.nanosec} \n")
        self.file_3.write(f"{self.count}, {self.count_1}  , {ouster_msg.header.stamp.sec}.{ouster_msg.header.stamp.nanosec} \n")

        self.file_4.write(f"{self.count},{self.count_1}  , {image_msg2.header.stamp.sec}.{image_msg2.header.stamp.nanosec} \n")
        self.file_5.write(f"{self.count},{self.count_1}  , {image_msg3.header.stamp.sec}.{image_msg3.header.stamp.nanosec} \n")
        self.file_6.write(f"{self.count},{self.count_1}  , {ouster_msg1.header.stamp.sec}.{ouster_msg1.header.stamp.nanosec} \n")

        if image_msg.header.stamp.sec < start_timestamp or image_msg.header.stamp.sec > end_timestamp:
            # Skip the message if it is outside the specified time interval
            self.count_1 = 0
            print("Skipping message due to timestamp constraints")
            print("Start timestamp:", start_timestamp)
            print("End timestamp:", end_timestamp)
            print("Current message timestamp:", image_msg.header.stamp.sec)
            return



        with open(self.folder_image_1 + filename + '_image_msg.pkl', 'wb') as f:
            pickle.dump(image_msg, f)
        with open(self.folder_image_2 + filename + '_image_msg.pkl', 'wb') as f:
            pickle.dump(image_msg1, f)
        with open(self.folder_image_3 + filename + '_image_msg.pkl', 'wb') as f:
            pickle.dump(image_msg2, f)
        with open(self.folder_image_4 + filename + '_image_msg.pkl', 'wb') as f:
            pickle.dump(image_msg3, f)
        with open(self.folder_pc + filename + '_pointcloud_msg.pkl', 'wb') as f:
            pickle.dump(ouster_msg, f)
        with open(self.folder_pc_1 + filename + '_pointcloud_msg.pkl', 'wb') as f:
            pickle.dump(ouster_msg1, f)

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv_image1 = self.bridge.imgmsg_to_cv2(image_msg1, desired_encoding='bgr8')
        undistorted_image_left = cv2.undistort(cv_image, self.matrix_coefficients_left,self.distortion_coefficients_left)
        undistorted_image_right = cv2.undistort(cv_image1, self.matrix_coefficients_right,self.distortion_coefficients_right)
        cv2.imwrite(self.folder_image_1 + filename + '.jpg', undistorted_image_left)
        cv2.imwrite(self.folder_image_2 + filename +'.jpg', undistorted_image_right)
        
        cv_image2 = self.bridge.imgmsg_to_cv2(image_msg2, desired_encoding='bgr8')
        cv_image3 = self.bridge.imgmsg_to_cv2(image_msg3, desired_encoding='bgr8')
        undistorted_image_left2 = cv2.undistort(cv_image2, self.matrix_coefficients_left2,self.distortion_coefficients_left2)
        undistorted_image_right3 = cv2.undistort(cv_image3, self.matrix_coefficients_right3,self.distortion_coefficients_right3)
        cv2.imwrite(self.folder_image_3 + filename + '.jpg', undistorted_image_left2)
        cv2.imwrite(self.folder_image_4 + filename +'.jpg', undistorted_image_right3)
        '''
        cv2.namedWindow("Image 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image 2", cv2.WINDOW_NORMAL)
        # Process the images (example: show them)
        cv2.imshow("Image 1", undistorted_image_left)
        cv2.imshow("Image 2", undistorted_image_right)

        cv2.namedWindow("Image 3", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image 4", cv2.WINDOW_NORMAL)
        # Process the images (example: show them)
        cv2.imshow("Image 3", undistorted_image_left2)
        cv2.imshow("Image 4", undistorted_image_right3)

        cv2.waitKey(1)  # Display images for a short period of time
        #print(ros2_numpy.point_cloud2.point_cloud2_to_array(ouster_msg).keys())

        '''
        pc_as_numpy_array = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(ouster_msg)['xyz'] )
        
        pc_as_numpy_array_intensity = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(ouster_msg)['intensity'] )
        pc_as_numpy_array = self.transformation_ouster(pc_as_numpy_array)
        
        pc_as_numpy_array_intensity = pc_as_numpy_array_intensity.squeeze()
        
        points = np.hstack([pc_as_numpy_array[:, :3], pc_as_numpy_array_intensity.reshape(-1, 1)])#.tolist()
        points = np.nan_to_num(points, nan=0.0)
        
        timestamp = float(f"{ouster_msg.header.stamp.sec}.{ouster_msg.header.stamp.nanosec:09d}")
        save_pcd_module.save_pcd(f"{self.folder_pc}{filename}.pcd", points.tolist(), timestamp)

        pc_as_numpy_array1 = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(ouster_msg1)['xyz'] )
        
        pc_as_numpy_array_intensity1 = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(ouster_msg1)['intensity'] )
        pc_as_numpy_array1 = self.transformation_ouster(pc_as_numpy_array1)
        pc_as_numpy_array_intensity1 = pc_as_numpy_array_intensity1.squeeze()
        points1 = np.hstack([pc_as_numpy_array1[:, :3], pc_as_numpy_array_intensity1.reshape(-1, 1)])#.tolist()
        points1 = np.nan_to_num(points1, nan=0.0)
        timestamp1 = float(f"{ouster_msg1.header.stamp.sec}.{ouster_msg1.header.stamp.nanosec:09d}")
        save_pcd_module.save_pcd(f"{self.folder_pc_1}{filename}.pcd", points1.tolist(), timestamp1)
        
        
        self.count_1 = self.count_1 + 1
        # Close all OpenCV windows  
        if self.count == 500:
            exit()


        
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
