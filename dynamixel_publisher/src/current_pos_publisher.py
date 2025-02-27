import rospy
import dynamixel_sdk as dxl
import math
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

class DynamixelCurrentBasedPositionPublisher:
    def __init__(self, device_name='/dev/ttyUSB0', baudrate=57600, dxl_ids=None):
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PWM_LIMIT = 36
        self.ADDR_CURRENT_LIMIT = 38
        self.ADDR_VELOCITY_LIMIT = 44
        self.ADDR_PROFILE_ACCELERATION = 108
        
        self.PROTOCOL_VERSION = 2.0
        self.DXL_RESOLUTION = 4096
        self.DEGREE_TO_RADIAN = math.pi / 180

        self.device_name = device_name
        self.baudrate = baudrate
        self.dxl_ids = dxl_ids if dxl_ids else [1,2,3,4,5,6,7,8]
        

        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)
        
        if not self.port_handler.openPort():
            rospy.logerr("Cannot open the port")
        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logerr("Cannot set the baudrate")

        self.initialized = False
        self.initial_positions = {
            1: 2048, 2: 2048, 3: 1024, 4: 2048,
            5: 2048, 6: 2048, 7: 2048, 8: 1024
        }

        for dxl_id in self.dxl_ids:
            self.set_operating_mode(dxl_id, 5)
            # self.set_pwm_limit(dxl_id, 442)
            self.set_current_limit(dxl_id, 1000)
            # self.set_velocity_limit(dxl_id, 1023)
            # self.set_acceleration_profile(dxl_id, 100)
            self.enable_torque(dxl_id, True)
        
        self.group_sync_read_position = dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, self.ADDR_PRESENT_POSITION, 4
        )
        self.group_sync_read_current = dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, self.ADDR_PRESENT_CURRENT, 2
        )
        
        for dxl_id in self.dxl_ids:
            self.group_sync_read_position.addParam(dxl_id)
            self.group_sync_read_current.addParam(dxl_id)
        
        self.position_publisher = rospy.Publisher('/tocabi/R_arm_pos', Float32MultiArray, queue_size=10)
        self.current_publisher = rospy.Publisher('/dynamixel_current', Float32MultiArray, queue_size=10)
        self.tracker_publisher = rospy.Publisher("TRACKERSTATUS", Bool, queue_size=1000)


    def set_operating_mode(self, dxl_id, mode):
        self.packet_handler.write1ByteTxOnly(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, mode)
        rospy.loginfo(f"Set ID {dxl_id} to operating mode {mode}")
    
    def enable_torque(self, dxl_id, enable):
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, int(enable))
        rospy.loginfo(f"Torque {'enabled' if enable else 'disabled'} for ID {dxl_id}")


    # def set_pwm_limit(self, dxl_id, pwm_limit):
    #     self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, self.ADDR_PWM_LIMIT, pwm_limit)
    

    def set_current_limit(self, dxl_id, current_limit):
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, self.ADDR_CURRENT_LIMIT, current_limit)
    

    # def set_velocity_limit(self, dxl_id, velocity_limit):
    #     self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_VELOCITY_LIMIT, velocity_limit)
    
    # def set_acceleration_profile(self, dxl_id, acceleration):
    #     self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PROFILE_ACCELERATION, acceleration)
    

    def set_goal_position(self, dxl_id, position):
        position = int(position)
        self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, position)
        # rospy.loginfo(f"Set goal position {position} for ID {dxl_id}")


    def read_position_radian(self):
        positions = {}
        for dxl_id in self.dxl_ids:
            data = self.group_sync_read_position.getData(dxl_id, self.ADDR_PRESENT_POSITION, 4)
            if data is not None:
                # pulse to degree (1 pulse = 0.088 deg)
                degree = data * (360.0 / self.DXL_RESOLUTION)
                # set the 180 degree as 0 radian, (0~180 deg: negative), (180~360: positive)
                if degree < 180.0:
                    radian = - (180.0 - degree) * self.DEGREE_TO_RADIAN
                elif degree > 180.0:
                    radian = (degree - 180.0) * self.DEGREE_TO_RADIAN
                else:
                    radian = 0.0
                positions[dxl_id] = radian
            else:
                rospy.logwarn(f"Cannot read {dxl_id} data")
        return positions
    
    def read_position(self):
        positions = {}
        
        dxl_comm_result = self.group_sync_read_position.txRxPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            rospy.logerr(f"Failed to read position: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return positions  

        for dxl_id in self.dxl_ids:
            if not self.group_sync_read_position.isAvailable(dxl_id, self.ADDR_PRESENT_POSITION, 4):
                rospy.logwarn(f"Position data not available for ID {dxl_id}")
                continue

            data = self.group_sync_read_position.getData(dxl_id, self.ADDR_PRESENT_POSITION, 4)
            positions[dxl_id] = data

        return positions
    

    def read_current(self):
        currents = {}
        self.group_sync_read_current.txRxPacket()
        for dxl_id in self.dxl_ids:
            data = self.group_sync_read_current.getData(dxl_id, self.ADDR_PRESENT_CURRENT, 2)
            if data is not None:
                if data > 32767: # 2의 보수 변환
                    data -= 65536
                # current_A = data * 0.00269  # 1 unit = 2.69mA
                current_A = data
                currents[dxl_id] = current_A
            else:
                rospy.logwarn(f"Cannot read current data for ID {dxl_id}")

        return currents
    

    def publish_data(self):
        rate = rospy.Rate(250)      # 250Hz
        while not rospy.is_shutdown():
            positions_rad = self.read_position_radian()
            positions = self.read_position()
            currents = self.read_current()

            if not self.initialized:
                for dxl_id in self.dxl_ids:
                    if dxl_id in self.initial_positions:
                        self.set_goal_position(dxl_id, self.initial_positions[dxl_id])
                self.initialized = True
            
            for dxl_id in self.dxl_ids:
                if dxl_id in positions:
                    self.set_goal_position(dxl_id, positions[dxl_id])

            self.tracker_publisher.publish(True)
            self.position_publisher.publish(Float32MultiArray(data=[positions_rad[dxl_id] for dxl_id in self.dxl_ids]))
            self.current_publisher.publish(Float32MultiArray(data=[currents[dxl_id] for dxl_id in self.dxl_ids]))
            
            rate.sleep()
    

    def __del__(self):
        self.port_handler.closePort()


if __name__ == '__main__':
    try:
        rospy.init_node('dynamixel_publisher')
        publisher = DynamixelCurrentBasedPositionPublisher()
        publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
