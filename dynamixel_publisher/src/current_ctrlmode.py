import rospy
import dynamixel_sdk as dxl
import math
import time
from std_msgs.msg import Float32MultiArray, Bool
from dynamixel_sdk import GroupSyncWrite


def DXL_LOBYTE(w):
    return w & 0xFF


def DXL_HIBYTE(w):
    return (w >> 8) & 0xFF


def DXL_LOWORD(l):
    return l & 0xFFFF


def DXL_HIWORD(l):
    return (l >> 16) & 0xFFFF


class DynamixelCurrentControl:
    def __init__(self, device_name="/dev/ttyUSB1", baudrate=57600, dxl_ids=None):
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_CURRENT_LIMIT = 38

        self.PROTOCOL_VERSION = 2.0
        self.DXL_RESOLUTION = 4096
        self.DEGREE_TO_RADIAN = math.pi / 180
        self.PULSE_TO_RADIAN = (2 * math.pi) / self.DXL_RESOLUTION

        self.device_name = device_name
        self.baudrate = baudrate
        self.dxl_ids = (
            dxl_ids
            if dxl_ids
            else [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
        )

        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)

        self.offsets = {dxl_id: None for dxl_id in self.dxl_ids}

        if not self.port_handler.openPort():
            rospy.logerr("Cannot open the port")
        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logerr("Cannot set the baudrate")

        for dxl_id in self.dxl_ids:
            # current ctrl mode : 0
            # position ctrl mode : 3
            # current based position ctrl mode : 5
            self.set_operating_mode(dxl_id, 0)  # Current-based Position Control Mode
            self.set_current_limit(dxl_id, 500)  # Set current limit
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
            "/dynamixel_currents", Float32MultiArray, queue_size=10
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

    def set_current_limit(self, dxl_id, current_limit):
        self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_CURRENT_LIMIT, current_limit
        )

    def set_goal_current(self, dxl_id, current):
        self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_GOAL_CURRENT, current
        )

    def read_position(self):
        positions = {}
        self.group_sync_read_position.txRxPacket()
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

    def read_current(self):
        currents = {}
        self.group_sync_read_current.txRxPacket()
        for dxl_id in self.dxl_ids:
            data = self.group_sync_read_current.getData(
                dxl_id, self.ADDR_PRESENT_CURRENT, 2
            )
            currents[dxl_id] = data if data is not None else 0
        return currents

    def publish_data(self):
        positions = self.read_position()
        currents = self.read_current()
        self.position_publisher.publish(
            Float32MultiArray(data=[positions[dxl_id] for dxl_id in self.dxl_ids])
        )
        self.current_publisher.publish(
            Float32MultiArray(data=[currents[dxl_id] for dxl_id in self.dxl_ids])
        )

    def __del__(self):
        self.port_handler.closePort()


if __name__ == "__main__":
    try:
        rospy.init_node("dynamixel_current_control")
        controller = DynamixelCurrentControl()
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            controller.publish_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
