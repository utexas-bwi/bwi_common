import time
import rospy

from std_srvs.srv import Empty
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def _stop(robot):
    stop = MoveBaseGoal()
    stop.target_pose.header.frame_id = '{}/level_mux_map'.format(robot)
    stop.target_pose.pose = rospy.wait_for_message('/{}/odom'.format(robot), Odometry, timeout=1).pose.pose
    return stop
def _get_pose_with_cov(robot):
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = '{}/level_mux_map'.format(robot)
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose = rospy.wait_for_message('/{}/odom'.format(robot), Odometry, timeout=1).pose.pose
    init_pose.pose.covariance[0] = init_pose.pose.covariance[7] = 1e-3
    init_pose.pose.covariance[35] = (3.14/24)**2
    return init_pose
if __name__ == '__main__':
    rospy.init_node("reset_by_hand")
    reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    rate = rospy.Rate(2)
    # Regular Robot
    move_regular = SimpleActionClient('marvin/move_base', MoveBaseAction)
    clear_regular = rospy.ServiceProxy('/marvin/move_base/clear_costmaps', Empty)
    localize_regular = rospy.Publisher('/marvin/initialpose', PoseWithCovarianceStamped, queue_size=1)
    move_regular.wait_for_server()
    # Polite Robot
    move_polite = SimpleActionClient('roberto/move_base', MoveBaseAction)
    clear_polite = rospy.ServiceProxy('/roberto/move_base/clear_costmaps', Empty)
    localize_polite = rospy.Publisher('/roberto/initialpose', PoseWithCovarianceStamped, queue_size=1)
    move_polite.wait_for_server()

    # =============Reset Protocol===================
    # Stop robots
    print("Stop robots...")
    tic = time.time()
    move_regular.send_goal_and_wait(_stop('marvin'))
    print("\tmarvin: {:.2f}s:".format(time.time()-tic))
    tic = time.time()
    move_polite.send_goal_and_wait(_stop('roberto'))
    print("\troberto: {:.2}s".format(time.time()-tic))

    rate.sleep()
    # Reset environment
    print("Reset world...")
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_proxy()
    except (rospy.ServiceException) as e:
        print("/gazebo/reset_world service call failed")
        rate.sleep()
    print("Localize robots...")
    rospy.sleep(1)
    # Publish localization topics to both robots
    localize_polite.publish( _get_pose_with_cov('roberto') )
    localize_regular.publish( _get_pose_with_cov('marvin') )
    rospy.sleep(2) # Wait for system to do localization process

    # Clear costmaps
    print("Clear costmaps...")
    rospy.wait_for_service('/marvin/move_base/clear_costmaps')
    try:
        clear_regular()
    except (rospy.ServiceException) as e:
        print("/marvin/move_base/clear_costmaps service call failed")
    rospy.wait_for_service('/roberto/move_base/clear_costmaps')
    try:
        clear_polite()
    except (rospy.ServiceException) as e:
        print("/roberto/move_base/clear_costmaps service call failed")

    print("Now You Can Test Hallway Passing!")
