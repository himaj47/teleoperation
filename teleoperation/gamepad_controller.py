import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool
from sensor_msgs.msg import Joy

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')

        self.disc_position_pub_ = self.create_publisher(Int32, "/disc_position", 10)
        self.shoot_angle_pub_ = self.create_publisher(Int32, "/shoot_angle", 10)
        self.fly_wheels_speed_pub_ = self.create_publisher(Float32, "/fly_wheels_speed", 10)
        self.pneumatics_state_pub_ = self.create_publisher(Bool, "/pneumatics", 10)
        # self.create_subscription(Int32, "/disc_encoder", self.disc_enocder_callback, 10)
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.disc_position_max_ = 500
        # self.disc_encoder_min_ = 500
        self.shoot_angle_position_max_ = 500
        # self.shoot_angle_encoder_min_ = 500
        self.flywheel_curr_speed_ = 0.0
        self.step_ = 0.05
        self.pressed_ = True

    def joy_callback(self, msg:Joy):
        leftHatY = msg.axes[1]  # to adjust shoot angle
        # leftHatX = msg.axes[0]
        rightHatX = msg.axes[3] # to adjust disc rotation 

        enable_angle_adjustments = msg.buttons[5]   # to enable angle adjustments
        increment_flywheel_speed = msg.buttons[3]
        decrement_flywheel_speed = msg.buttons[0]
        enable_pneumatic = msg.buttons[1]

        # mappings
        set_disc_position = int(rightHatX * self.disc_position_max_)
        set_shoot_angle = int(leftHatY * self.shoot_angle_position_max_)

        if enable_angle_adjustments:
            angle_adjustment_msg = Int32()
            vel_msg = Float32()

            if rightHatX > 0.0:
                angle_adjustment_msg.data = set_disc_position
                self.disc_position_pub_.publish(angle_adjustment_msg)

            if leftHatY > 0.0:
                angle_adjustment_msg.data = set_shoot_angle
                self.shoot_angle_pub_.publish(angle_adjustment_msg)

            if increment_flywheel_speed:
                self.flywheel_curr_speed_ += self.step_

            if decrement_flywheel_speed:
                self.flywheel_curr_speed_ -= self.step_

            if enable_pneumatic and self.pressed_:
                state = Bool()
                state.data = True
                self.pneumatics_state_pub_.publish(state)
                self.pressed_ = False
            
            elif not enable_pneumatic:
                self.pressed_ = True

            vel_msg.data = self.flywheel_curr_speed_
            self.fly_wheels_speed_pub_.publish(vel_msg)
            

def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()