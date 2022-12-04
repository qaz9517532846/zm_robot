import rclpy
from zm_robot_programing.zm_robot_action import zm_robot_cmd

def main(args=None):
    rclpy.init(args=args)
    zm_robot = zm_robot_cmd()
    zm_robot.move_map(1.0, 1.0, 0.0)
    zm_robot.move_map(1.0, 0.0, 0.0)
    zm_robot.move_base(-1.0, 0.0, 0.0)
    exit(0)


if __name__ == '__main__':
    main()