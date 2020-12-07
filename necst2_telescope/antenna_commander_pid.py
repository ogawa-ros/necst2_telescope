#!/usr/bin/env python3

node_name = 'antenna_commander_pid'

import numpy as np
import time
import rclpy
from std_msgs.msg import Float64

class antenna_feedback(object):

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

        self.p_coeff = double(self.node.declare_parameter("p_coeff").value)
        self.i_coeff = double(self.node.declare_parameter("i_coeff").value)
        self.d_coeff = double(self.node.declare_parameter("d_coeff").value)

        self.hensa_stock = [0] * self.i_ave_num

        self.gear_ratio = double(self.node.declare_parameter("gear_ratio").value)
        self.pulseper360deg = double(self.node.declare_parameter("pulseper360deg").value)
        self.pulse_a = double(self.node.declare_parameter("pulse_a").value)
        self.pulse_b = double(self.node.declare_parameter("pulse_b").value)

        self.MOTOR_MAXSTEP = double(self.node.declare_parameter("MOTOR_MAXSTEP").value)
        self.MOTOR_MAXSPEED = double(self.node.declare_parameter("MOTOR_MAXSPEED").value)
        
	map_azel = {'az': 'y', 'el': 'x'}
        topic_name = {'to'   : f'pyinterface/pci7415/rsw0/azel_xy/speed_cmd',
                      'cur'  : f'opu1p85m/azel/current_speed',
                      'tar'  : f'opu1p85m/azel/target_speed',
                      'hensa': f'opu1p85m/azel/pid_hensa',
                      'from1': f'opu1p85m/azel/cmd2',
                      'from2': f'dev/HEIDENHAIN/ND287/azel'}
        self.topic_to = self.node.create_publisher(Float64, topic_name['to'], 1)
        self.topic_cur = self.node.create_publisher(Float64, topic_name['cur'], 1)
        self.topic_tar = self.node.create_publisher(Float64, topic_name['tar'], 1)
        self.topic_hensa = self.node.create_publisher(Float64, topic_name['hensa'], 1)
        topic_from1 = self.node.create_subscription(Float64, topic_name['from1'], self.antenna_feedback, 1)
        topic_from2 = self.node.create_subscription(Float64, topic_name['from2'], self.antenna_encoder, 1)
        pass

    def antenna_feedback(self, command):
        # deg/sec
        self.target_deg = command.data

        if self.t_past == 0.0:
            self.t_past = time.time()
        else:
            pass
        self.t_now = time.time()

        speed = self.calc_pid()

        # update
        self.pre_hensa = self.target_deg - self.encoder_deg
        self.pre_deg = self.target_deg
        self.enc_before = self.encoder_deg
        self.t_past = self.t_now

        # deg -> pulse
        speed = speed * self.gear_ratio / 360.0 * (self.pulseper360deg * (self.pulse_b / self.pulse_a))

        # limit of acceleration
        if abs(speed - self.speed_d) < self.MOTOR_MAXSTEP:
            self.speed_d = speed
        else:
            if (speed - self.speed_d) < 0:
                a = -1.0
            else:
                a = 1.0
            self.speed_d += a * self.MOTOR_MAXSTEP

        # limit of max speed
        if self.speed_d > self.MOTOR_MAXSPEED:
            self.speed_d = self.MOTOR_MAXSPEED
        if self.speed_d < -self.MOTOR_MAXSPEED:
            self.speed_d = -self.MOTOR_MAXSPEED

        if self.lock == True:
            self.speed_d = 0.0
        else:
            pass

        msg_cmd = Float64()
        msg_cmd.data = -1 * self.speed_d
        self.topic_to.publish(msg_cmd)

        return

    
    def antenna_encoder(self, status):
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
        if np.abs(dhensa) > 1:
            dhensa = 0.0

        if (self.encoder_deg - self.enc_before) != 0.0:
            self.current_speed = (self.encoder_deg - self.enc_before) / (self.t_now - self.t_past)
        
        if self.pre_deg == 0: # for first move
            target_speed = 0.0
        else:
            target_speed = (self.target_deg - self.pre_deg) / (self.t_now - self.t_past)
        
        try:
            self.ihensa = sum(self.hensa_stock) / len(self.hensa_stock)
        except:
            self.ihensa = 0.0

        # PID
        rate = target_speed + self.p_coeff * hensa + self.i_coeff * self.ihensa * (self.t_now - self.t_past) + self.d_coeff * dhensa / (self.t_now - self.t_past)

        msg_target = Float64()
        msg_target.data = target_speed
        msg_current = Float64()
        msg_current.data = self.current_speed
        msg_hensa = Float64()
        msg_hensa.data = hensa

        self.topic_tar.publish(msg_target)
        self.topic_cur.publish(msg_current)
        self.topic_hensa.publish(msg_hensa)

        return rate


def main(args=None):
    rclpy.init(args=args)
    feedback = antenna_feedback()
    rclpy.spin(feedback.node)

    feedback.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
