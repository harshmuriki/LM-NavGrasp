import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import struct
import tf2_ros
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs

rospy.init_node('depth_image_segmenter', anonymous=True)
tf2_buffer = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(tf2_buffer)

def transform_pointcloud_to_image(point_cloud_msg, tf2_buffer, camera_info_msg):

    height, width = 334, 434
    K = camera_info_msg.K
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]
    # Extract RGB and XYZ data from the pointcloud
    pc_data = pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))
    points = np.array(list(pc_data))
    
    # Separate XYZ and RGB
    xyz = points[:, :3]
    rgb_float = points[:, 3]
    
    # Extract RGB using the provided method
    rgb = []
    for float_rgb in rgb_float:
        s = struct.pack('>f', float_rgb)
        i = struct.unpack('>l', s)[0]
        r = (i >> 16) & 0xFF
        g = (i >> 8) & 0xFF
        b = i & 0xFF
        rgb.append([r, g, b])
    rgb = np.array(rgb)

    xyz_transformed = []
    for point in xyz:
        point_stamped = PointStamped()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        transform = tf2_buffer.lookup_transform("camera_color_optical_frame", point_cloud_msg.header.frame_id, point_cloud_msg.header.stamp, rospy.Duration(1.0))
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        xyz_transformed.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

    xyz_transformed = np.array(xyz_transformed)
    print(xyz_transformed)
    # Transform points to camera frame
    # try:
    #     transform = tf2_buffer.lookup_transform("camera_color_optical_frame", point_cloud_msg.header.frame_id, point_cloud_msg.header.stamp, rospy.Duration(1.0))
    #     xyz_transformed = tf2_geometry_msgs.do_transform_point(xyz, transform)
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #     rospy.logerr(f"TF Error: {e}")
    #     return None

    # Project 3D points to 2D image plane
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])  # Replace with actual camera intrinsics
    points_2d = np.dot(camera_matrix, xyz_transformed.T).T
    points_2d = points_2d[:, :2] / points_2d[:, 2:]

    # Create image and populate with RGB values
    image = np.zeros((height, width, 3), dtype=np.uint8)  # Replace height and width with actual image dimensions
    for (x, y), color in zip(points_2d.astype(int), rgb):
        if 0 <= x < width and 0 <= y < height:
            image[y, x] = color
    print("saved")
    cv2.imwrite("image.jpg", image)

    return image


def save_rgb_image(segmented_points, camera_info_msg, image_width, image_height):
    """
    Save a 2D RGB image from segmented 3D points.

    :param segmented_points: List of 3D points with RGB values [(x, y, z, r, g, b)].
    :param camera_info_msg: CameraInfo message containing camera intrinsics.
    :param image_width: Width of the output image.
    :param image_height: Height of the output image.
    """
    K = np.array(camera_info_msg.K).reshape(3, 3)
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

    for (x, y, z, r, g, b) in segmented_points:
        u = int((K[0, 0] * x / z) + K[0, 2])
        v = int((K[1, 1] * y / z) + K[1, 2])
        if 0 <= u < image_width and 0 <= v < image_height:
            image[v, u] = (b, g, r)  # OpenCV uses BGR format

    cv2.imwrite("segmented_rgb_image.jpg", image)


def segment_point_cloud_v1(segmented_pixels, point_cloud_msg, camera_info_msg, image_width=434, image_height=334):
    K = np.array(camera_info_msg.K).reshape(3, 3)
    points_list = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)

    segmented_points = []
    for point in points_list:
        x, y, z, rgb = point
        s = struct.pack('>f', rgb)
        i = struct.unpack('>l', s)[0]
        r = (i >> 16) & 0xFF
        g = (i >> 8) & 0xFF
        b = i & 0xFF
        segmented_points.append((x, y, z, r, g, b))

    # Save the RGB image
    save_rgb_image(segmented_points, camera_info_msg, image_width, image_height)

    # Create and publish the segmented point cloud
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = point_cloud_msg.header.frame_id

    fields = [
        pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
    ]

    # Pack RGB values into a single uint32
    packed_points = [(x, y, z, struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]) for x, y, z, r, g, b in segmented_points]

    segmented_cloud_msg = pc2.create_cloud(header, fields, packed_points)

    point_cloud_pub = rospy.Publisher('/segmented_point_cloud', PointCloud2, queue_size=10)
    point_cloud_pub.publish(segmented_cloud_msg)
    rospy.loginfo("Published segmented point cloud")

    return segmented_cloud_msg


def depth_callback(depth_image_msg):
    coordinates = []
    with open('/home/hello-robot/catkin_ws/src/lm_navgrasp/src/coordinates.txt', 'r') as f:
        for line in f:
            x, y = map(int, line.strip().split(','))
            coordinates.append((x, y))

    segmented_pixels = np.asarray(coordinates)
    print("inside")
    camera_info_msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    print("inside2")
    # segmented_cloud = segment_point_cloud_v1(segmented_pixels, depth_image_msg, camera_info_msg)
    image = transform_pointcloud_to_image(depth_image_msg, tf2_buffer, camera_info_msg)
    print(image)

def listener():
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, depth_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()




# asdqwsdqdq


    # def update(self, point_cloud_msg, tf2_buffer):
    #     self.max_height_im.clear()
    #     cloud_time = point_cloud_msg.header.stamp
    #     cloud_frame = point_cloud_msg.header.frame_id
    #     point_cloud = rn.numpify(point_cloud_msg)
    #     only_xyz = False
    #     if only_xyz:
    #         xyz = rn.point_cloud2.get_xyz_points(point_cloud)
    #         self.max_height_im.from_points_with_tf2(xyz, cloud_frame, tf2_buffer)
    #     else: 
    #         rgb_points = rn.point_cloud2.split_rgb_field(point_cloud)
    #         self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, tf2_buffer)
    #     obstacle_im = self.max_height_im.image == 0
    #     self.updated = True

        # def from_rgb_points_with_tf2(self, rgb_points, points_frame_id, tf2_buffer, points_timestamp=None, timeout_s=None):
        # # points should be a numpy array with shape = (N, 3) where N
        # # is the number of points. So it has the following structure:
        # # points = np.array([[x1,y1,z1], [x2,y2,z2]...]). The points
        # # should be specified with respect to the coordinate system
        # # defined by points_frame_id.
        
        # points_to_voi_mat, timestamp = self.voi.get_points_to_voi_matrix_with_tf2(points_frame_id, tf2_buffer, lookup_time=points_timestamp, timeout_s=timeout_s)

        # if points_to_voi_mat is not None: 
        #     self.from_rgb_points(points_to_voi_mat, rgb_points)

        #     if points_timestamp is None:
        #         if timestamp is None: 
        #             self.last_update_time = rospy.Time.now()
        #         else:
        #             self.last_update_time = timestamp
        #     else:
        #         self.last_update_time = points_timestamp
        # else:
        #     rospy.logwarn('ROSMaxHeightImage.from_rgb_points_with_tf2: failed to update the image likely due to a failure to lookup the transform using TF2. points_frame_id = {0}, points_timestamp = {1}, timeout_s = {2}'.format(points_frame_id, points_timestamp, timeout_s))
