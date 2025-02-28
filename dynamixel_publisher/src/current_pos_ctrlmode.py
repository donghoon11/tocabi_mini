import rospy
import dynamixel_sdk as dxl
import math
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from dynamixel_sdk import GroupSyncWrite


def DXL_LOBYTE(w):
    return w & 0xFF


def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF


def DXL_LOWORD(l):
    return l & 0xFFFF


def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF


class DynamixelCurrentBasedPositionPublisher:
    def __init__(
        self, device_name="/dev/ttyUSB0", baudrate=57600, dxl_ids=None, dxl_ids_r=None
    ):
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
        self.dxl_ids = (
            dxl_ids
            if dxl_ids
            else [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
        )

        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)

        self.PULSE_TO_RADIAN = (2 * math.pi) / self.DXL_RESOLUTION
        self.offsets = {dxl_id: None for dxl_id in self.dxl_ids}

        if not self.port_handler.openPort():
            rospy.logerr("Cannot open the port")
        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logerr("Cannot set the baudrate")

        self.initialized = True  # False : initializing ready pose
        self.initial_positions = {
            1: 2048,
            2: 2048,
            3: 1024,
            4: 2048,
            5: 2048,
            6: 2048,
            7: 2048,
            8: 1024,
            9: 2048,
            10: 2048,
            11: 1024,
            12: 2048,
            13: 2048,
            14: 2048,
            15: 2048,
            16: 1024,
        }

        for dxl_id in self.dxl_ids:
            self.set_operating_mode(dxl_id, 5)
            self.set_current_limit(dxl_id, 1000)
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

        self.position_publisher = rospy.Publisher(
            "/tocabi/Arm_pose", Float32MultiArray, queue_size=10
        )
        self.current_publisher = rospy.Publisher(
            "/dynamixel_current", Float32MultiArray, queue_size=10
        )
        self.tracker_publisher = rospy.Publisher(
            "/TRACKERSTATUS", Bool, queue_size=1000
        )

    def set_operating_mode(self, dxl_id, mode):
        self.packet_handler.write1ByteTxOnly(
            self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, mode
        )
        rospy.loginfo(f"Set ID {dxl_id} to operating mode {mode}")

    def enable_torque(self, dxl_id, enable):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, int(enable)
        )
        rospy.loginfo(f"Torque {'enabled' if enable else 'disabled'} for ID {dxl_id}")

    def set_current_limit(self, dxl_id, current_limit):
        self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_CURRENT_LIMIT, current_limit
        )

    def set_goal_position(self, dxl_id, position):
        position = int(position)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, position
        )

    def read_position_radian(self):
        positions = {}

        for dxl_id in self.dxl_ids:
            raw_data = self.group_sync_read_position.getData(
                dxl_id, self.ADDR_PRESENT_POSITION, 4
            )
            if raw_data is None:
                continue

            position_radian = (raw_data % self.DXL_RESOLUTION) * self.PULSE_TO_RADIAN

            if self.offsets[dxl_id] is None:
                self.offsets[dxl_id] = position_radian

            data = position_radian - self.offsets[dxl_id]

            if data > math.pi:
                data -= 2 * math.pi
            elif data < -math.pi:
                data += 2 * math.pi

            positions[dxl_id] = data
        return positions

    def read_position(self):
        positions = {}

        dxl_comm_result = self.group_sync_read_position.txRxPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            rospy.logerr(
                f"Failed to read position: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            return positions

        for dxl_id in self.dxl_ids:
            if not self.group_sync_read_position.isAvailable(
                dxl_id, self.ADDR_PRESENT_POSITION, 4
            ):
                rospy.logwarn(f"Position data not available for ID {dxl_id}")
                continue

            data = self.group_sync_read_position.getData(
                dxl_id, self.ADDR_PRESENT_POSITION, 4
            )
            positions[dxl_id] = data

        return positions

    def read_current(self):
        currents = {}
        self.group_sync_read_current.txRxPacket()
        for dxl_id in self.dxl_ids:
            data = self.group_sync_read_current.getData(
                dxl_id, self.ADDR_PRESENT_CURRENT, 2
            )
            if data is not None:
                if data > 32767:  # 2의 보수 변환
                    data -= 65536
                # current_A = data * 0.00269  # 1 unit = 2.69mA
                current_A = data
                currents[dxl_id] = current_A
            else:
                rospy.logwarn(f"Cannot read current data for ID {dxl_id}")

        return currents

    def initialize_positions(self):
        if not self.initialized:
            sync_write_goal_pos = GroupSyncWrite(
                self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, 4
            )

            for dxl_id in self.dxl_ids:
                if dxl_id in self.initial_positions:
                    param_goal_position = [
                        DXL_LOBYTE(DXL_LOWORD(self.initial_positions[dxl_id])),
                        DXL_HIBYTE(DXL_LOWORD(self.initial_positions[dxl_id])),
                        DXL_LOBYTE(DXL_HIWORD(self.initial_positions[dxl_id])),
                        DXL_HIBYTE(DXL_HIWORD(self.initial_positions[dxl_id])),
                    ]
                    sync_write_goal_pos.addParam(dxl_id, param_goal_position)

            sync_write_goal_pos.txPacket()  # 모든 모터에 한 번에 명령 전송
            sync_write_goal_pos.clearParam()  # 메모리 해제
            self.initialized = True

    def update_positions(self, positions):
        """Dynamixel 목표 위치를 업데이트하는 함수"""
        sync_write_goal_pos = GroupSyncWrite(
            self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, 4
        )

        for dxl_id in self.dxl_ids:
            if dxl_id in positions:
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(positions[dxl_id])),
                    DXL_HIBYTE(DXL_LOWORD(positions[dxl_id])),
                    DXL_LOBYTE(DXL_HIWORD(positions[dxl_id])),
                    DXL_HIBYTE(DXL_HIWORD(positions[dxl_id])),
                ]
                sync_write_goal_pos.addParam(dxl_id, param_goal_position)

        sync_write_goal_pos.txPacket()  # 모든 모터에 한 번에 명령 전송
        sync_write_goal_pos.clearParam()  # 메모리 해제

    def publish_data(self):

        positions_rad = self.read_position_radian()
        positions = self.read_position()

        if not self.initialized:
            for dxl_id in self.dxl_ids:
                if dxl_id in self.initial_positions:
                    self.set_goal_position(dxl_id, self.initial_positions[dxl_id])
            self.initialized = True

        for dxl_id in self.dxl_ids:
            if dxl_id in positions:
                self.set_goal_position(dxl_id, positions[dxl_id])

        if not self.initialized:
            self.initialize_positions()

        self.update_positions(positions)

        self.tracker_publisher.publish(True)
        self.position_publisher.publish(
            Float32MultiArray(data=[positions_rad[dxl_id] for dxl_id in self.dxl_ids])
        )

    def __del__(self):
        self.port_handler.closePort()


if __name__ == "__main__":
    try:
        rospy.init_node("dynamixel_publisher")
        publisher = DynamixelCurrentBasedPositionPublisher()
        rate = rospy.Rate(250)  # 250Hz
        while not rospy.is_shutdown():
            start_time = time.time()
            publisher.publish_data()
            rate.sleep()
            end_time = time.time()

            print(end_time - start_time)
    except rospy.ROSInterruptException:
        pass
