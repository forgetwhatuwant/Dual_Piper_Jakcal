#!/usr/bin/env python3
import rospy

from lifting_ctrl.srv import LiftMotorSrv  # 升降杆控制服务  

class LIFTER:
    def __init__(self):
        self.lifter_client = rospy.ServiceProxy('/lifter_1/LiftingMotorService', LiftMotorSrv)

# int64 val 
# int64 mode
# ---
# int64 state

    def move(self,val,mode):
        rospy.wait_for_service('/lifter_1/LiftingMotorService')
        try:
            #将末端执行器移动至输入的位姿
            self.lifter_client(val,mode)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

# test code
if __name__ == '__main__':
    # rospy.init_node('image_saver', anonymous=True)
    lifter = LIFTER() 
    lifter.move(480, 0)


