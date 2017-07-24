from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
import time
import math
from pymavlink import mavutil


mode_str = '' #global
class uav():
    '''all the common funcs. ned to wgs in meters, dist to nextwp, dist btw two wgs points, takeoff and
    arm'''
    def __init__(self, aTargetAltitude, connection_str):

        self.ini_alt = aTargetAltitude
        self.uav_endpoint = connection_str

        self.uav_lat = 0.0
        self.uav_lon = 0.0
        self.uav_alt = 0.0
        self.uav_north = 0.0
        self.uav_east = 0.0
        self.uav_down = 0.0
        self.dist_z = 0.0
        self.elev_curr = 0.0
        self.relative_alt = 0.0
        self.uav_home_location = 0


    def uav_connect(self):
        #todo check if command and clear causing mission update problem
        print 'Connecting to vehicle on: %s' % self.uav_endpoint
        self.vehicle = connect(self.uav_endpoint, wait_ready=True)
        self.cmds = self.vehicle.commands
        self.cmds.clear()

        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print "waiting for home location to set....."
            self.uav_home_location = self.vehicle.home_location
            print "\n Home location set: {0}".format(self.vehicle.home_location)
            self.home_location_check()
        # return self.vehicle
    def home_location_check(self):
        valid_wgs_loc = LocationGlobal(self.uav_home_location.lat, self.uav_home_location.lon, self.uav_home_location.alt)
        if(type(valid_wgs_loc) is LocationGlobal):
            print "valid home location on vehicle"
        else:
            raise Exception("INVALID LOCATION LOCATION TYPE")


    def uav_disconnect(self):
        print "Shutting down Vehicle interface"
        self.vehicle.close()

    def get_location_metres(self, original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(
            math.pi * original_location.lat / 180))  # cos of lat bcz each degree of lon(az) changes at which lat(parallel) we are

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt)

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint == 0:
            return None
        missionitem = self.vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)  # lat, lon wrt WGS84, ALT wrt home location
        self.distancetopoint = self.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return self.distancetopoint

    def arm_and_takeoff(self):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(self.ini_alt)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: relative to home position %s " % self.vehicle.location.global_relative_frame.alt
            print "Altitude: relative to MSL %s" % self.vehicle.location.global_frame.alt
            if self.vehicle.location.global_relative_frame.alt >= self.ini_alt* 0.95:  # Trigger just below target alt.
                print "Reached target altitude"
                break
            time.sleep(1)

    def master_slave_dist_wgs(self, master_loc, slave_loc):

        return self.get_distance_metres(slave_loc, master_loc)
    #todo implement master_slave_dist_ned vector if required

    #todo convert axis_dist and elev angle to properties
    def axis_dist(self, home_loc, curr_loc):
        '''pass lat, lon, in WGS alt is not requried'''
        self.dist_z = self.get_distance_metres(home_loc, curr_loc)
        return self.dist_z

    def elevation_current(self, home_loc, curr_loc):

        self.relative_alt = (curr_loc.alt - self.ini_alt)
        self.elev_curr = math.degrees(math.atan2(self.relative_alt, self.axis_dist(home_loc, curr_loc)))
        return self.elev_curr

    def observer_global_relative_frame(self, *args):

        self.uav_lat = args[2].lat
        self.uav_lon = args[2].lon
        self.uav_alt = args[2].alt
        # self.print_wgs_frame()
        # print "{0}".format(len(args))
        #
        # print "ARGS::0>>>{0}".format(args[0].version)
        # print "ARGS::2>>>{0}".format(args[2].lat)
        # print "ARGS::1>>>{0}".format(args[1].__str__())


    def add_observer_global_relative_frame(self):
        self.vehicle.add_attribute_listener('location.global_relative_frame', self.observer_global_relative_frame)

    def remove_observer_global_relative_frame(self):
        self.vehicle.remove_attribute_listener('location.global_relative_frame', self.observer_global_relative_frame)

    def observer_local_frame(self, *args):
        self.uav_north = args[2].north
        self.uav_east = args[2].east
        self.uav_down = args[2].down

    def add_observer_local_frame(self):
        self.vehicle.add_attribute_listener('location.local_frame', self.observer_local_frame)

    def remove_observer_local_frame(self):
        self.vehicle.remove_attribute_listener('location.local_frame', self.observer_local_frame)