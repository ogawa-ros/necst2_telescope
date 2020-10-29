#!/usr/bin/env python3
node_name = "coordinate_calculation"

import time
import rclpy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String

from astropy.utils.iers import conf
from astropy.time import Time
from astropy.time import TimeDelta
from astropy.coordinates import FK5
from astropy.coordinates import EarthLocation
from astropy.coordinates import SkyCoord
from astropy.coordinates import AltAz
import astropy.constants
import astropy.units as u

class coordinate_calc(object):

    press = 1000
    temp  = 0
    humid = 0.5
    freq = 230 #GHz

    latitude = 35.940874
    longitude = 138.472153
    height = 1386
    nobeyama = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)

    kisa_path = "/home/exito/ros/src/necst-1p85m2019/lib/kisa.dat"

    azel = []

    def __init__(self):
        self.node = rclpy.create_node(node_name)

        self.node.create_subscription(String , '/necst/telescope/coordinate_cmd',self.recieve_coordinate_cmd)
        self.node.create_subscription(Bool   , "stop_cmd" ,self.recieve_stop_cmd,1)
        self.node.create_subscription(Float64, "press"    ,self.recieve_pressure, 1)
        self.node.create_subscription(Float64, "temp"     ,self.recieve_temprature, 1)
        self.node.create_subscription(Float64, "humid"    ,self.recieve_humidity, 1)
        self.node.create_subscription(Float64, "freq"    ,self.recieve_frequency, 1)
        self.node.create_subscription(Float64, "az"    ,self.recieve_enc_az, 1)
        self.node.create_subscription(Float64, "el"    ,self.recieve_enc_el, 1)

        self.pub_scan_flag = self.node.create_publisher(Bool, "scan_flag", 1)
        self.pub_azel      = self.node.create_publisher(Float64MultiArray, "apparent_azel", 1)

        coordinate_cmd = "{'time':12000000,'x':180,'y':45,'frame':azel,'offset_az':10,'offset_el':10,scan_flag:True}")
        #'frame':azel or fk5(wcs) or planet

        self.read_kisa()
        self.node.create_timer(0.1, self.publish_azel)


    def recieve_pressure(self,q):
        self.press = q.data

    def recieve_temprature(self,q):
        self.temp = q.data

    def recieve_humidity(self,q):
        self.humid = q.data

    def recieve_frequency(self,q):
        self.freq = q.data

    def recieve_enc_az(self,q):
        self.enc_az = q.data

    def recieve_enc_el(self,q):
        self.enc_el = q.data

    def recieve_stop_cmd(self,q):
        self.azel = []
        t = time.time()
        self.azel2 = [t,self.enc_az,self.enc_el,False]
        pass


    def recieve_coordinate_cmd(self,q):
        cmd_dict = eval(q.data)
        t = cmd_dict["time"] #unix time
        x = cmd_dict["x"]
        y = cmd_dict["y"]
        frame  = cmd_dict["frame"]
        offset_az = cmd_dict["offset_az"]
        offset_el = cmd_dict["offset_el"]
        scan_flag = cmd_dict["scan_flag"]

        if frame = azel:
            self.regist_azel_cmd(t,x,y,offset_az,offset_el,scan_flag)
        elif frame = fk5:
            self.regist_wcs_cmd(t,x,y,offset_az,offset_el,frame,scan_flag)
        else:
            self.regist_planet_cmd(t,planet,offset_az,offset_el,scan_flag)


    def regist_planet_cmd(self,t,offset_az,offset_el,frame,scan_flag):
        on_coord = astropy.coordinates.get_body(location=self.nobeyama,time=t,body=frame) #tの単位要確認
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.freq*u.GHz)).to('micron')
        altaz = on_coord.transform_to(AltAz(obstime=t))
        regist_list = [altaz.obstime.to_value("unix"),altaz.az.deg+offset_az, altaz.alt.deg+offset_el,scan_flag]
        self.azel.append(regist_list)

    def regist_wcs_cmd(self,t,x,y,offset_az,offset_el,frame,scan_flag):
        on_coord = SkyCoord(x, y,frame=frame, unit=(u.deg, u.deg))
        on_coord.location = self.nobeyama
        on_coord.pressure = self.press*u.hPa
        on_coord.temperature = self.temp*u.deg_C
        on_coord.relative_humidity = self.humid
        on_coord.obswl = (astropy.constants.c/(self.freq*u.GHz)).to('micron')
        altaz = on_coord.transform_to(AltAz(obstime=t))
        regist_list = [altaz.obstime.to_value("unix"),altaz.az.deg+offset_az, altaz.alt.deg+offset_el,scan_flag]
        self.azel.append(regist_list)


    def regist_azel_cmd(self,t,x,y,offset_az,offset_el,scan_flag):
        regist_list =[t,x+offset_az,y+offset_el,scan_flag]
        self.azel.append(regist_list)


    def read_kisa(self):
        fkisa = open(self.kisa_path,"r")
        kisa = fkisa.readlines()
        self.a1 = float(kisa[0])
        self.a2 = float(kisa[1])
        self.a3 = float(kisa[2])
        self.b1 = float(kisa[3])
        self.b2 = float(kisa[4])
        self.b3 = float(kisa[5])
        self.g1 = float(kisa[6])
        self.c1 = float(kisa[7])
        self.c2 = float(kisa[8])
        self.d1 = float(kisa[9])
        self.d2 = float(kisa[10])
        self.e1 = float(kisa[11])
        self.e2 = float(kisa[12])

    def calculate_kisa(self,az,el):
        az = math.radians(az)
        el = math.radians(el)
        el2 = math.degrees(el/180*math.pi)

        cos_az = math.cos(az)
        sin_az = math.sin(az)
        cos_el = math.cos(el)
        sin_el = math.sin(el)
        cos_azel = math.cos(az-el)
        sin_azel = math.sin(az-el)
        pi = math.pi

        ## d_az[deg] , d_el[deg] ##
        d_az = self.a1*sin_el + self.a2 + self.a3*cos_el + self.b1*sin_az*sin_el - self.b2*cos_az*sin_el
        d_el = self.b1*cos_az + self.b2*sin_az + self.b3 + self.g1*el2

        #if self.optobs == False:
        if self.freq > 300000000: #MHz=300THz
            d_az = d_az + self.c1*sin_azel + self.c2*cos_azel + self.d1+ self.e1*cos_el - self.e2*sin_el
            d_el = d_el + self.c1*cos_azel - self.c2*sin_azel + self.d2+ self.e1*sin_el + self.e2*cos_el
        else:
            pass

        ### convert to encoder offset on the horizon ###
        d_az = d_az / cos_el
        ### apply the correction values  ###
        azaz = azel[0] + d_az
        elel = azel[1] + d_el
        return azaz , elel


    def publish_azel(self):
        try:#if len＝０
            azel = self.azel.pop(0)
            t = azel[0]
            az = azel[1]
            el = azel[2]
            scan_flag = azel[3]
            azaz, elel = calculate_kisa(az,el)
            self.azel2 = [t, azaz, elel, scan_flag]
        except:
            pass

        t  = self.azel2[0]
        while True:
            if t < time.time():
                #Float64MultiArrayなげる
                msg = Float64()
                msg.data = adafsdf
                self.pub_azel.publish(msg)
                self.pub_scan_flag.publish(scan_flag)
                break
            else:
                time.sleep(0.001)
                continue
        pass


def main(args=None):
    rclpy.init(args=args)
    coordinate = coordinate_calc()
    rclpy.spin(coordinate.node)
    coordinate.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
