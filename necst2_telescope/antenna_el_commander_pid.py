#!/usr/bin/env python3

node_name = 'antenna_el_commander_pid'

import math
import time
import rclpy
from std_msgs.msg import Float64

class antenna_el_feedback(object):

    speed_d = 0.0
    pre_deg = 0.0
    pre_hensa = 0.0
    enc_before = 0.0
    ihensa = 0.0
    i_ave_num = 10
    t_now = t_past = 0.0
    current_speed = 0.0

    encoder_deg = 0.0

    lock = False


    def __init__(self):
        self.node = rclpy.create_node(node_name)
        self.node.declare_parameter("p_coeff")
        self.node.declare_parameter("i_coeff")
        self.node.declare_parameter("d_coeff")
        self.p_coeff = self.node.get_parameter("p_coeff").get_parameter_value().string_value
        self.i_coeff = self.node.get_parameter("i_coeff").get_parameter_value().string_value
        self.d_coeff = self.node.get_parameter("d_coeff").get_parameter_value().string_value

        self.hensa_stock = [0] * self.i_ave_num

        self.node.declare_parameter("gear_ratio")
        self.node.declare_parameter("pulseper360deg")
        self.node.declare_parameter("pulse_a")
        self.node.declare_parameter("pulse_b")
        self.gear_ratio = self.node.get_parameter("gear_ratio").get_parameter_value().string_value
        self.pulseper360deg = self.node.get_parameter("pulseper360deg").get_parameter_value().string_value
        self.pulse_a = self.node.get_parameter("pulse_a").get_parameter_value().string_value
        self.pulse_b = self.node.get_parameter("pulse_b").get_parameter_value().string_value

        self.node.declare_parameter("MOTOR_MAXSTEP")
        self.node.declare_parameter("MOTOR_el_MAXSPEED")
        self.MOTOR_MAXSTEP = self.node.get_parameter("MOTOR_MAXSTEP").get_parameter_value().string_value
        self.MOTOR_el_MAXSPEED = self.node.get_parameter("MOTOR_el_MAXSPEED").get_parameter_value().string_value
        
        topic_name = {'to'   : '/opu1p85m/el_speed',
                      'cur'  : '/opu1p85m/el_current_speed',
                      'tar'  : '/opu1p85m/el_target_speed',
                      'hensa': '/opu1p85m/el_pid_hensa',
                      'from1': '/opu1p85m/el_cmd2',
                      'from2': '/opu1p85m/el'}
        self.topic_to = rclpy.node.create_publisher(Float64, topic_name['to'], 1)
        self.topic_cur = rclpy.node.create_publisher(Float64, topic_name['cur'], 1)
        self.topic_tar = rclpy.node.create_publisher(Float64, topic_name['tar'], 1)
        self.topic_hensa = rclpy.node.create_publisher(Float64, topic_name['hensa'], 1)
        topic_from1 = rclpy.node.create_subscription(Float64, topic_name['from1'], self.antenna_el_feedback, 1)
        topic_from2 = rclpy.node.create_subscription(Float64, topic_name['from2'], self.antenna_el_encoder, 1)
        pass

    def antenna_el_feedback(self, command):
        self.target_deg = command.data

        if self.t_past == 0.0:
            self.t_past = time.time()
        else:
            pass
        self.t_now = time.time()

        speed = self.calc_pid(self.ihensa)

        # update
        self.pre_hensa = self.target_deg - self.encoder_deg
        self.pre_deg = self.target_deg
        self.enc_before = self.encoder_deg
        self.t_past = self.t_now

        # deg -> pulse
        speed = speed * self.gear_ratio / 360 * (self.pulseper360deg * (self.pulse_b / self.pulse_a))

        # limit of acceleration
        if abs(speed - self.speed_d) < self.MOTOR_MAXSTEP:
            self.speed_d = speed
        else:
            if (speed - self.speed_d) < 0:
                a = -1
            else:
                a = 1
            self.speed_d += a * self.MOTOR_MAXSTEP

        # limit of max speed
        if self.speed_d > self.MOTOR_el_MAXSPEED:
            self.speed_d = self.MOTOR_el_MAXSPEED
        if self.speed_d < -self.MOTOR_el_MAXSPEED:
            self.speed_d = -self.MOTOR_el_MAXSPEED

        command_speed = self.speed_d

        if self.lock == True:
            self.speed_d = 0.0
            self.topic_to.publish(0.0)
            return
        else:
            self.topic_to.publish(command_speed)
        return

    
    def antenna_el_encoder(self, status):
        self.encoder_deg = status.data
        return

    def calc_pid(self):
        """
        DESCRIPTION
        ===========
        This function determine az&el speed for antenna
        """

        # calculate ichi_hensa
        hensa = self.target_deg - self.encoder_deg

        self.hensa_stock.append(hensa)
        self.hensa_stock = self.hensa_stock[1:]

        dhensa = hensa - self.pre_hensa
        if math.fabs(dhensa) > 1:
            dhensa = 0

        if (self.encoder_deg - self.enc_before) != 0.0:
            self.current_speed = (self.encoder_deg - self.enc_before) / (self.t_now - self.t_past)
        
        if self.pre_deg == 0: # for first move
            target_speed = 0
        else:
            target_speed = (self.target_deg - self.pre_deg) / (self.t_now - self.t_past)
        
        try:
            self.ihensa = sum(self.hensa_stock) / len(self.hensa_stock)
        except:
            self.ihensa = 0

        # PID
        rate = target_speed + self.p_coeff*hensa + self.i_coeff*self.ihensa*(self.t_now-self.t_past) + self.d_coeff*dhensa/(self.t_now-self.t_past)
        self.topic_tar.publish(target_speed)
        self.topic_cur.publish(self.current_speed)
        self.topic_hensa.publish(hensa)

        return rate


def main(args=None):
    rclpy.init(args=args)
    feedback = antenna_el_feedback()
    rclpy.spin(feedback.node)

    feedback.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
