from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
from pymavlink import mavutil
from threading import Thread
from math import sqrt, pow, acos, atan2, cos, degrees, asin, sin, tan
import time
import math
from uav_module import uav
import uav_module


class uav_slave(uav, Thread):
    def __init__(self, endpoint_str, uav_alt, shared_q, groundspeed, m_s_dist, alt_offset, m_home_loc_q):
        Thread.__init__(self)
        uav.__init__(self, uav_alt,endpoint_str)
        self.master_q = shared_q
        self.mst_slv_dist = m_s_dist
        self.alt_off = alt_offset

        # global mode_str

        self.g_speed = groundspeed

        self.bearing = 0.0
        self.hm_loc_q_m = m_home_loc_q

    def run(self):
        self.uav_connect()
        self.add_observer_global_relative_frame()
        # time.sleep(2)
        self.arm_and_takeoff()
        self.vehicle.groundspeed = self.g_speed
        time.sleep(8) #todo timing is really imp., 2~4 sec for 2 swarm but 6~8 sec for 3 swarm, this time may increase due to resource
        self.uav_guide()
        self.uav_disconnect()

    def get_bearing(self, aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        self.bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if self.bearing < 0:
            self.bearing += 360.00
        return self.bearing

    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def target_local_ned(self, north, east, down):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def uav_guide(self):
        '''of load master ned coord from the Q, check if it is string or not
        if string master_mode RTL is send break the loop and land'''

        # mode_str = ''
        # global mode_str
        axis_dist = 0.0

        home_loc_m = self.vehicle.home_location #to be overwritten
        if(self.hm_loc_q_m.empty() != True):
            home_loc_m = self.hm_loc_q_m.get()


        while (uav_module.mode_str != "RTL"):

            # if (self.mode_str_q.empty() != True):
            #     mode_str = self.mode_str_q.get()


            if(self.master_q.empty() != True):
                try:
                    ned_list = self.master_q.get() #get and remove the item from the q
                except: #master_q would be shutting down for one slave inst.
                    print "could not get the item in ned list"
                    break

                d_curr_m = ned_list.pop()
                e_curr_m = ned_list.pop()
                n_curr_m = ned_list.pop()
                print "THREAD: N: %f E: %f D(abs): %f" % (n_curr_m, e_curr_m, abs(d_curr_m))
                mag_vec_m = (sqrt(pow(n_curr_m, 2) + pow(e_curr_m, 2) + pow(d_curr_m, 2))) - 10
                sph_alt = abs(d_curr_m) - 10
                try:

                    sph_ang = asin((sph_alt / mag_vec_m))
                    axis_dist = sph_alt / tan(sph_ang)
                except:
                    print "calculation error {0}, {1}, {2}".format(sph_alt, mag_vec_m, axis_dist)

                Un = (n_curr_m / mag_vec_m)
                Ue = (e_curr_m / mag_vec_m)
                Ud = (d_curr_m / mag_vec_m)

                est_mst_slv_dist_n = axis_dist + self.mst_slv_dist * cos(math.radians(90))
                est_mst_slv_dist_e = axis_dist + self.mst_slv_dist * sin(math.radians(90))

                SUn = Un * (est_mst_slv_dist_n)
                SUe = Ue * (est_mst_slv_dist_e)
                # SUn = Un * (abs(alt_off))
                # SUe = Ue * (abs(alt_off))
                SUd = Ud * (abs(self.alt_off))
                if (self.alt_off > 0):

                    slave_n = n_curr_m + (SUn)  # put slave n and e coord for assesment
                    slave_e = e_curr_m + (SUe)
                    slave_d = d_curr_m + SUd
                else:
                    slave_n = n_curr_m - (SUn)  # put slave n and e coord for assesment
                    slave_e = e_curr_m - (SUe)
                    slave_d = d_curr_m - SUd

                self.target_local_ned(slave_n, slave_e, slave_d)

                s_curr_loc = LocationGlobalRelative(self.uav_lat, self.uav_lon, self.uav_alt)
                s_yaw_loc = self.get_bearing(home_loc_m, s_curr_loc)
                if(s_yaw_loc > 180):
                    s_yaw_loc = s_yaw_loc - 180
                    self.condition_yaw(s_yaw_loc)
                else:
                    s_yaw_loc = s_yaw_loc + 180
                    self.condition_yaw(s_yaw_loc)


        time.sleep(1)
        self.vehicle.mode = VehicleMode("LAND")
        print "MASTER MODE@: {0}".format(uav_module.mode_str)
        time.sleep(3)
        print "SLAVE MODE: {0}".format(self.vehicle.mode)
        self.vehicle.remove_attribute_listener('location.global_relative_frame', self.observer_global_relative_frame)