import ros_numpy as rn

cloud_frame = point_cloud_msg.header.frame_id
point_cloud = rn.numpify(point_cloud_msg)
rgb_points = rn.point_cloud2.split_rgb_field(point_cloud)
self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, tf2_buffer)
