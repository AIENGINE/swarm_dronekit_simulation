from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
from pymavlink import mavutil
from threading import Thread
from collections import deque
from math import sqrt, pow, acos, atan2, cos, degrees, asin, sin, tan
import time
import math
from uav_module import uav
import uav_module



class uav_master(uav, Thread):

    def __init__(self, endpoint_str, uav_ini_alt, radius, turns, groundspeed, **kwargs):
        '''valid Queue name in kwagrs are ['shared_q', 'hm_loc_q','shared_q_1', 'hm_loc_q_1']'''
        Thread.__init__(self)
        uav.__init__(self, uav_ini_alt, endpoint_str)
        #todo list to track parameter change on startup and during the mission, so logs can be created
        self.radius = radius
        self.turns = turns
        self.groundspeed = groundspeed
        self.uav_init_alt = uav_ini_alt

        # global mode_str

        self.master_ned_list = deque()

        self.master_q_len = 0
        if(len(kwargs) == 2 or len(kwargs) == 4):
            self.master_q_len = len(kwargs)
            self.q_setup(kwargs)
        elif(len(kwargs)>0 or len(kwargs)>4):
            raise Exception ('required Queues are not passed')
        else:
            self.master_q_len = len(kwargs) #zero len here, normal operation single master

    def q_setup(self, kwargs):

        '''two Queue obj's are required for each slave, '''

        self.master_q = kwargs.get('shared_q', False)
        self.home_loc_q = kwargs.get('hm_loc_q', False)

        self.master_q_1 = kwargs.get('shared_q_1', False)
        self.home_loc_q_1 = kwargs.get('hm_loc_q_1', False)

        q_allowed = ['shared_q', 'hm_loc_q','shared_q_1', 'hm_loc_q_1']
        for qk, qv in kwargs.items():
            if(qk not in q_allowed):
                raise TypeError("pass only allowed names for the queue")

            if(type(qv).__name__ != 'Queue'):
                raise TypeError('Queue type error!! pass Queue type object')

    def q_close(self):
        '''Queue clean up routine'''
        if (self.master_q_len == 2 or self.master_q_len == 4):
            self.master_q.close()
            self.home_loc_q.close()
        if(self.master_q_len == 4):
            self.master_q_1.close()
            self.home_loc_q_1.close()


    def __str__(self):
        return "{0}".format(self.__class__.__name__)

    def observer_global_relative_frame(self, *args):

        self.uav_lat = args[2].lat
        self.uav_lon = args[2].lon
        self.uav_alt = args[2].alt

        # self.print_wgs_frame()

    def print_wgs_frame(self):

        print "{0}, {1}, {2}".format(self.__str__(), self.uav_lat, self.uav_lon)

    def observer_local_frame(self, *args):

        self.uav_north = args[2].north
        self.uav_east = args[2].east
        self.uav_down = args[2].down

        #ned_share will only be activated if number of slaves are 1 or 2
        if(self.master_q_len == 2 or self.master_q_len == 4):
            self.ned_share()


    def set_roi(self, location):
        """
        Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a
        specified region of interest (LocationGlobal).
        The vehicle may also turn to face the ROI.

        For more information see:
        http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
        """

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            location.lat,
            location.lon,
            location.alt
        )
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def add_mission_cmds(self, point, crnt_alt):

        self.cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
                    0, 0, 0,
                    0, 0, 0, point.lat, point.lon, crnt_alt))
        return self.cmds

    def wp_sphere(self):
        # no. of turns or turns = 90/Elevation Resolution cli input

        ini_height = self.ini_alt
        crnt_loc = self.vehicle.location.global_relative_frame
        total_points = 4 * self.turns
        phi = 2 * math.pi / 4
        theta = phi / 4 / self.turns

        for pnt in range(total_points):
            north = self.radius * math.cos(theta * pnt) * math.cos(phi * pnt)
            east = self.radius * math.cos(theta * pnt) * math.sin(phi * pnt)
            alt = ini_height * (self.radius * math.sin(theta * pnt))
            location_earth_pnt_curr = self.get_location_metres(crnt_loc, north, east)
            self.cmds = self.add_mission_cmds(location_earth_pnt_curr, alt)


        print "......................UPLOADING SPLINE WP SPHERE..................."
        time.sleep(2)
        self.cmds.upload()

    def auto_navigate(self):
        # global mode_str
        self.home_loc = self.uav_home_location

        self.vehicle.groundspeed = self.groundspeed
        self.vehicle.mode = VehicleMode('AUTO')
        time.sleep(0.3)
        print "UAV_MASTER: MODE::{0}".format(self.vehicle.mode)
        self.set_roi(self.home_loc)

        while True:
            nextwaypoint = self.vehicle.commands.next
            print '\rDistance to waypoint (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint())

            self.uav_loc = LocationGlobalRelative(self.uav_lat, self.uav_lon, self.uav_alt)
            try:
                self.elev_curr_m = self.elevation_current(self.home_loc, self.uav_loc)
                print "HOME LOC VEH: {0}".format(self.home_loc)

                print "CALLBACK: N: %f E: %f D: %f"%\
                      (self.uav_north, self.uav_east, self.uav_down)
                print "GPS_RELATIVE_FRAME: lat: %f lon: %f alt: %f"%\
                (self.uav_lat, self.uav_lon, self.uav_alt)

                sph_alt = abs(self.uav_down) - self.ini_alt
                est_magvec_relative = (sqrt(pow(self.uav_north, 2) + pow(self.uav_east, 2) + pow(self.uav_down, 2))) - 10
                sph_ang = asin((sph_alt / est_magvec_relative))
                gd_center = sph_alt / tan(sph_ang)
                print "Distance to axis: %f" %(gd_center)

                print "\rELEVATION ANGLE IN DEGREES %f" % (self.elev_curr_m)
            except:
                print "Value error with relative alt {0}, distz {1} ".format(self.relative_alt, self.dist_z)

            wp_count = self.vehicle.commands.count
            wp_last = wp_count - 1
            if (nextwaypoint == wp_last):
                print "LAST IN SPLINE WP REACHED"
                break
            time.sleep(1)
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(2)
        print "UAV_MASTER: RETURN TO LAUNCH:: {0}".format(self.vehicle.mode)
        uav_module.mode_str = self.vehicle.mode
        self.remove_observer_local_frame()
        self.remove_observer_global_relative_frame()
        time.sleep(1)


    def ned_share(self):
        #todo ned_share is launched in call back mode so it will run in any case even, HANDLE PROBLEM!!
        self.master_ned_list.append(self.uav_north)
        self.master_ned_list.append(self.uav_east)
        self.master_ned_list.append(self.uav_down)
        if(type(self.master_q).__name__ == 'Queue'):
            self.master_q.put(self.master_ned_list)
        if (type(self.master_q_1).__name__ == 'Queue'):
            self.master_q_1.put(self.master_ned_list)

    def homeloc_share(self):
        '''two home location queues for slave 1 and slave 2.'''
        if(type(self.home_loc_q).__name__ == 'Queue'):
            self.home_loc_q.put(self.uav_home_location) #todo hm_loc_q close
        if(type(self.home_loc_q_1).__name__ == 'Queue'):
            self.home_loc_q_1.put(self.uav_home_location)


    def run(self):
        self.uav_connect()
        self.add_observer_global_relative_frame()
        self.add_observer_local_frame()

        if(self.master_q_len == 2 or self.master_q_len == 4):
            self.homeloc_share()

        self.wp_sphere()

        if(self.master_q_len == 2 or self.master_q_len == 4):
            self.ned_share()

        self.arm_and_takeoff()
        time.sleep(1)
        self.auto_navigate()
        self.q_close()
        self.uav_disconnect()

