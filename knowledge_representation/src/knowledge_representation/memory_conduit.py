import rospy
from knowledge_representation.srv import GetCloud, GetCloudRequest, GetCloudResponse

class MemoryConduit:
    def __init__(self):
        pass

    def get_facing_cloud(self, include_ground=False):
        service = rospy.ServiceProxy("/memory_conduit_proxy/get_facing_cloud", GetCloud)
        try:
            service.wait_for_service(4)
            res = service()
            return res.cloud
        except rospy.ServiceException as e:
            print(e.message)
            return None