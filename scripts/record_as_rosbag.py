import subprocess
import rospy
import os
from rospkg import RosPack

def get_topics_from_ros_param(param_name):
    if rospy.has_param(param_name):
        topics = rospy.get_param(param_name)
        rospy.loginfo(f"record_as_rosbag: ROS parameter '{param_name}' successfully read: {topics}")
        return topics
    else:
        raise rospy.ROSException(f"ROS parameter '{param_name}' not found.")

def main():
    rospy.init_node('record_as_rosbag', anonymous=True)

    # get the parameters
    topics = get_topics_from_ros_param("/debug_and_profile_helper/record_list")
    
    try:
        output_bag_path = get_topics_from_ros_param("/debug_and_profile_helper/record_bag_path")
    except rospy.ROSException:
        rospy.logwarn("record_as_rosbag: Parameter '/debug_and_profile_helper/record_bag_path' not found. Using default path.")
        rospack = RosPack()  # Correctly instantiate RosPack
        package_path = rospack.get_path('debug_and_profile_helper')
        output_bag_path = f"{package_path}/recordings/"
    
    output_bag_name = get_topics_from_ros_param("/debug_and_profile_helper/record_bag_name")
    if not output_bag_name:
        print(f"Error: No bag name found in parameter /debug_and_profile_helper/record_bag_name.")
        raise rospy.ROSException(f"No bag name found in parameter /debug_and_profile_helper/record_bag_name.")

    try:
        allow_overwrite = get_topics_from_ros_param(rospy.get_name() + "/allow_overwrite")
    except rospy.ROSException:
        rospy.logwarn("Parameter '/debug_and_profile_helper/allow_overwrite' not found. Using default value: False.")
        allow_overwrite = False

    # if the output bag path does not exist, create it
    output_bag_path = output_bag_path.rstrip('/') + '/'
    if not os.path.exists(output_bag_path):
        try:
            os.makedirs(output_bag_path)
            rospy.loginfo(f"record_as_rosbag: Path does not exist. Created directory: {output_bag_path}")
        except OSError as e:
            rospy.logerr(f"record_as_rosbag: Failed to create directory {output_bag_path}: {e}")
            raise rospy.ROSException(f"Failed to create directory {output_bag_path}: {e}")
    
    # put the path together (also fix the missing / if possible)
    output_bag_path = output_bag_path + output_bag_name.rstrip('.bag') + '.bag'
    rospy.loginfo(f"record_as_rosbag: Output bag path: {output_bag_path}")

    # check if the output bag path already exists, decide the reaction depending on the allow_overwrite parameter
    if os.path.exists(output_bag_path):
        if allow_overwrite:
            rospy.logwarn(f"record_as_rosbag: Output bag file already exists: {output_bag_path}. Overwriting...")
        else:
            rospy.logerr(f"record_as_rosbag: Output bag file already exists: {output_bag_path}. Not allowed to overwrite.")
            raise rospy.ROSException(f"Output bag file already exists: {output_bag_path}. Not allowed to overwrite.")

    # error if topics is empty
    if not topics:
        print(f"Error: No topics found in parameter /debug_and_profile_helper/record_list.")
        raise rospy.ROSException(f"No topics found in parameter /debug_and_profile_helper/record_list.")
    else:
        print(f"Recording the following topics: {topics}")

    # run the rosbag record command
    command = ['rosbag', 'record', '-O', output_bag_path] + topics
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error while recording topics: {e}")
        raise rospy.ROSException(f"Failed to record topics: {e}")

if __name__ == "__main__":
    main()