import rospy
from std_msgs.msg import Float32
import dynamixel_sdk as dxl
import math

class DynamixelCurrentBasedPositionPublisher:
    
    def __init__(self, device_name='/dev/ttyUSB0', baudrate=57600, dxl_ids=None):
        # Control Table
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.PROTOCO_VERSION = 2.0
        self.DXL_RESOLUTION = 4096  # XL330-M077-T resolution (0~4095)
        self.DEGREE_TO_RAIDAN = math.pi / 180
        
        self.device_name = device_name
        self.baudrate = baudrate

        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCO_VERSION)

        if dxl_ids is None:
            dxl_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        
        self.dxl_ids = dxl_ids

        # 포트 열기
        if not self.port_handler.openPort():
            rospy.logerr("Cannot open the port")

        # 통신 속도 설정
        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logerr("Cannot set the baudrate")

        # 동작 모드 설정 및 초기 위치 설정
        initial_positions = {
            1: 2048,
            2: 2048,
            # 3: 1024,
            3: 512,
            4: 2048,
            5: 2048,
            6: 2048,
            7: 2048,
            8: 1024
        }

        for dxl_id in self.dxl_ids:
            self.set_operating_mode(dxl_id, 5)  # 5: Current-based Position Control Mode
            self.enable_torque(dxl_id, True)   # 토크 활성화
            if dxl_id in initial_positions:
                self.set_goal_position(dxl_id, initial_positions[dxl_id])  # Set initial position

        self.group_sync_read_position = dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, self.ADDR_PRESENT_POSITION, 4)

        self.group_sync_read_current = dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, self.ADDR_PRESENT_CURRENT, 2)


        for dxl_id in self.dxl_ids:
            if not self.group_sync_read_position.addParam(dxl_id):
                rospy.logerr(f"Cannot add ID {dxl_id} to GroupSyncRead for position")

            if not self.group_sync_read_current.addParam(dxl_id):
                rospy.logerr(f"Cannot add ID {dxl_id} to GroupSyncRead for current")


        self.position_publishers = {dxl_id: rospy.Publisher(f'/dynamixel/{dxl_id}/position', Float32, queue_size=10) for dxl_id in self.dxl_ids}
        self.current_publishers = {dxl_id: rospy.Publisher(f'/dynamixel/{dxl_id}/current', Float32, queue_size=10) for dxl_id in self.dxl_ids}


    def set_operating_mode(self, dxl_id, mode):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            rospy.logwarn(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logwarn(self.packet_handler.getRxPacketError(dxl_error))
        rospy.loginfo(f"Set ID {dxl_id} to operating mode {mode}")


    def enable_torque(self, dxl_id, enable):
        value = 1 if enable else 0
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, value)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            rospy.logwarn(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logwarn(self.packet_handler.getRxPacketError(dxl_error))
        rospy.loginfo(f"Torque {'enabled' if enable else 'disabled'} for ID {dxl_id}")


    def set_goal_position(self, dxl_id, position):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, position)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            rospy.logwarn(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logwarn(self.packet_handler.getRxPacketError(dxl_error))
        rospy.loginfo(f"Set goal position {position} for ID {dxl_id}")


    def read_position(self):
        positions = {}
        self.group_sync_read_position.txRxPacket()
        for dxl_id in self.dxl_ids:
            data = self.group_sync_read_position.getData(dxl_id, self.ADDR_PRESENT_POSITION, 4)
            if data is not None:
                # pulse to degree (1 pulse = 0.088 deg)
                degree = data * (360.0 / self.DXL_RESOLUTION)
                # set the 180 degree as 0 radian, (0~180 deg: negative), (180~360: positive)
                if degree < 180.0:
                    radian = - (180.0 - degree) * self.DEGREE_TO_RAIDAN
                elif degree > 180.0:
                    radian = (degree - 180.0) * self.DEGREE_TO_RAIDAN
                else:
                    radian = 0.0
                positions[dxl_id] = radian
            else:
                rospy.logwarn(f"Cannot read {dxl_id} data")
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
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            positions = self.read_position()
            currents = self.read_current()
            for dxl_id in self.dxl_ids:
                if dxl_id in positions:
                    self.position_publishers[dxl_id].publish(positions[dxl_id])
                if dxl_id in currents:
                    self.current_publishers[dxl_id].publish(currents[dxl_id])
            rate.sleep()

    def _del_(self):
        self.port_handler.clearPort()

if __name__ == '__main__':
    try:
        rospy.init_node('dynamixel_current_based_position_publisher')
        publisher = DynamixelCurrentBasedPositionPublisher()
        publisher.publish_data()
    except rospy.ROSInterruptException: