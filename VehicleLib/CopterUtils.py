import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

class Copter():

    def __init__(self, connection_string=None, vehicle=None):
        """ Initialize the object
        Use either the provided vehicle object or the connections tring to connect to the autopilot
        
        Input:
            connection_string       - the mavproxy style connection string, like tcp:127.0.0.1:5760
                                      default is None
            vehicle                 - dronekit vehicle object, coming from another instance (default is None)
        
        
        """
        
        #---- Connecting with the vehicle, using either the provided vehicle or the connection string
        if not vehicle is None:
            self.vehicle    = vehicle
            print("Using the provided vehicle")
        elif not connection_string is None:
            
            print("Connecting with vehicle...")
            self._connect(connection_string)
        else:
            raise("ERROR: a valid dronekit vehicle or a connection string must be supplied")
            return
            
        self._setup_listeners()

        self.airspeed           = 0.0       #- [m/s]    airspeed
        self.groundspeed        = 0.0       #- [m/s]    ground speed
        
        self.pos_lat            = 0.0       #- [deg]    latitude
        self.pos_lon            = 0.0       #- [deg]    longitude
        self.pos_alt_rel        = 0.0       #- [m]      altitude relative to takeoff
        self.pos_alt_abs        = 0.0       #- [m]      above mean sea level
        
        self.att_roll_deg       = 0.0       #- [deg]    roll
        self.att_pitch_deg      = 0.0       #- [deg]    pitch
        self.att_heading_deg    = 0.0       #- [deg]    magnetic heading
        
        self.wind_dir_to_deg    = 0.0       #- [deg]    wind direction (where it is going)
        self.wind_dir_from_deg  = 0.0       #- [deg]    wind coming from direction
        self.wind_speed         = 0.0       #- [m/s]    wind speed
        
        self.climb_rate         = 0.0       #- [m/s]    climb rate
        self.throttle           = 0.0       #- [ ]      throttle (0-100)
        
        self.ap_mode            = ''        #- []       Autopilot flight mode
        
        self.mission            = self.vehicle.commands #-- mission items
        
        self.location_home      = LocationGlobalRelative(0,0,0) #- LocationRelative type home
        self.location_current   = LocationGlobalRelative(0,0,0) #- LocationRelative type current position
        
    def _connect(self, connection_string):      #-- (private) Connect to Vehicle
        """ (private) connect with the autopilot
        
        Input:
            connection_string   - connection string (mavproxy style)
        """
        self.vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=60)
        self._setup_listeners()
        
    def _setup_listeners(self):                 #-- (private) Set up listeners
        #----------------------------
        #--- CALLBACKS
        #----------------------------
        if True:    
            #---- DEFINE CALLBACKS HERE!!!
            @self.vehicle.on_message('ATTITUDE')   
            def listener(vehicle, name, message):          #--- Attitude
                self.att_roll_deg   = math.degrees(message.roll)
                self.att_pitch_deg  = math.degrees(message.pitch)
                self.att_heading_deg = math.degrees(message.yaw)%360
                
            @self.vehicle.on_message('GLOBAL_POSITION_INT')       
            def listener(vehicle, name, message):          #--- Position / Velocity                                                                                                             
                self.pos_lat        = message.lat*1e-7
                self.pos_lon        = message.lon*1e-7
                self.pos_alt_rel    = message.relative_alt*1e-3
                self.pos_alt_abs    = message.alt*1e-3
                self.location_current = LocationGlobalRelative(self.pos_lat, self.pos_lon, self.pos_alt_rel)
                
                
            @self.vehicle.on_message('VFR_HUD')
            def listener(vehicle, name, message):          #--- HUD
                self.airspeed       = message.airspeed
                self.groundspeed    = message.groundspeed
                self.throttle       = message.throttle
                self.climb_rate     = message.climb 
                
            @self.vehicle.on_message('WIND')
            def listener(vehicle, name, message):          #--- WIND
                self.wind_speed         = message.speed
                self.wind_dir_from_deg  = message.direction % 360
                self.wind_dir_to_deg    = (self.wind_dir_from_deg + 180) % 360
                        
            
        return (self.vehicle)
        print(">> Connection Established")

          
        
    def is_armed(self):                         #-- Check whether uav is armed
        """ Checks whether the UAV is armed
        
        """
        return(self.vehicle.armed)
        
    def arm(self):                              #-- arm the UAV
        """ Arm the UAV
        """
        self.vehicle.armed = True
        
    def disarm(self):                           #-- disarm UAV
        """ Disarm the UAV
        """
        self.vehicle.armed = False

    def set_airspeed(self, speed):              #--- Set target airspeed
        """ Set uav airspeed m/s
        """
        self.vehicle.airspeed = speed
        
    def set_ap_mode(self, mode):                #--- Set Autopilot mode
        """ Set Autopilot mode
        """
        time_0 = time.time()
        try:
            tgt_mode    = VehicleMode(mode)
        except:
            return(False)
            
        while (self.get_ap_mode() != tgt_mode):
            self.vehicle.mode  = tgt_mode
            time.sleep(0.2)
            if time.time() < time_0 + 5:
                return (False)

        return (True)
        
    def get_ap_mode(self):                      #--- Get the autopilot mode
        """ Get the autopilot mode
        """
        self._ap_mode  = self.vehicle.mode
        return(self.vehicle.mode)
        
    def clear_mission(self):                    #--- Clear the onboard mission
        """ Clear the current mission.
        
        """
        cmds = self.vehicle.commands
        self.vehicle.commands.clear()
        self.vehicle.flush()

        # After clearing the mission you MUST re-download the mission from the vehicle
        # before vehicle.commands can be used again
        # (see https://github.com/dronekit/dronekit-python/issues/230)
        self.mission = self.vehicle.commands
        self.mission.download()
        self.mission.wait_ready()

    def download_mission(self):                 #--- download the mission
        """ Download the current mission from the vehicle.
        
        """
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready() # wait until download is complete.  
        self.mission = self.vehicle.commands

    def mission_add_takeoff(self, takeoff_altitude=50, takeoff_pitch=15, heading=None):
        """ Adds a takeoff item to the UAV mission, if it's not defined yet
        
        Input:
            takeoff_altitude    - [m]   altitude at which the takeoff is considered over
            takeoff_pitch       - [deg] pitch angle during takeoff
            heading             - [deg] heading angle during takeoff (default is the current)
        """
        if heading is None: heading = self.att_heading_deg
        
        self.download_mission()
        #-- save the mission: copy in the memory
        tmp_mission = list(self.mission)
        
        print tmp_mission.count
        is_mission  = False
        if len(tmp_mission) >= 1:
            is_mission = True
            print("Current mission:")
            for item in tmp_mission:
                print item
            #-- If takeoff already in the mission, do not do anything
            
        if is_mission and tmp_mission[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print ("Takeoff already in the mission")
        else:
            print("Takeoff not in the mission: adding")
            self.clear_mission()
            takeoff_item = Command( 0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, takeoff_pitch,  0, 0, heading, 0,  0, takeoff_altitude)
            self.mission.add(takeoff_item)
            for item in tmp_mission:
                self.mission.add(item)
            self.vehicle.flush()
            print(">>>>>Done")

         

    def get_current_mission(self):
        """
        Downloads the mission and returns the wp list and number of WP 
        
        Input: 
            vehicle
            
        Return:
            n_wp, wpList
        """

        print "Downloading mission"
        download_mission(self)
        missionList = []
        n_WP        = 0
        for wp in vehicle.commands:
            missionList.append(wp)
            n_WP += 1 
            
        return n_WP, missionList
        

    def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
            self,            #--- vehicle object
            wp_Last_Latitude,   #--- [deg]  Target Latitude
            wp_Last_Longitude,  #--- [deg]  Target Longitude
            wp_Last_Altitude):  #--- [m]    Target Altitude
        """
        Upload the mission with the last WP as given and outputs the ID to be set
        """
        # Get the set of commands from the vehicle
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()

        # Save the vehicle commands to a list
        missionlist=[]
        for cmd in cmds:
            missionlist.append(cmd)

        # Modify the mission as needed. For example, here we change the
        wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                               wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
        missionlist.append(wpLastObject)

        # Clear the current mission (command is sent when we call upload())
        cmds.clear()

        #Write the modified mission and flush to the vehicle
        for cmd in missionlist:
            cmds.add(cmd)
        cmds.upload()
        
        return (cmds.count)    

    def ChangeMode(self, mode):
        while self.vehicle.mode != VehicleMode(mode):
                self.vehicle.mode = VehicleMode(mode)
                time.sleep(0.5)
        return True

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        if self.vehicle.armed and self.vehicle.location.global_relative_frame.alt > 2:
            print "\n\tVehicle armed and possible flying, aborting take off!\n"
            return
        print "Basic pre-arm checks"
        # Don't let the user try to fly autopilot is booting
        if self.vehicle.mode.name == "INITIALISING":
            print "Waiting for vehicle to initialise"
            time.sleep(1)
        while self.vehicle.gps_0.fix_type < 2:
            print "Waiting for GPS...:", self.vehicle.gps_0.fix_type
            time.sleep(1)
            
        print "Arming motors"
        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.armed   = True

        while not self.vehicle.armed:
            print "Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        try:
            while self.vehicle.mode.name=="GUIDED":
                print " -> Alt:", self.vehicle.location.global_relative_frame.alt
                if abs(self.vehicle.location.global_relative_frame.alt-aTargetAltitude) < 0.05: 
                    print "\n\tReached %0.1f m\n" % (aTargetAltitude)
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            print "Keyboard Interrupt on arm_and_takeoff."
            pass # do cleanup here

    def arm_and_takeoff_nogps(self,aTargetAltitude):
            """
            Arms vehicle and fly to aTargetAltitude without GPS data.
            """

            ##### CONSTANTS #####
            DEFAULT_TAKEOFF_THRUST = 0.7
            SMOOTH_TAKEOFF_THRUST = 0.6

            print("Basic pre-arm checks")
            # Don't let the user try to arm until autopilot is ready
            # If you need to disable the arming check,
            # just comment it with your own responsibility.
            while not self.vehicle.is_armable:
                print(" Waiting for vehicle to initialise...")
                time.sleep(1)


            print("Arming motors")
            # Copter should arm in GUIDED_NOGPS mode
            self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
            self.vehicle.armed = True

            while not vehicle.armed:
                print(" Waiting for arming...")
                self.vehicle.armed = True
                time.sleep(1)

            print("Taking off!")

            thrust = DEFAULT_TAKEOFF_THRUST
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                print(" Altitude: %f  Desired: %f" %
                      (current_altitude, aTargetAltitude))
                if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                    print("Reached target altitude")
                    break
                elif current_altitude >= aTargetAltitude*0.6:
                    thrust = SMOOTH_TAKEOFF_THRUST
                set_attitude(thrust = thrust)
                time.sleep(0.2)


    def go_to_alt(self, target):
        """
        Function that makes the vehicle travel to an specific lat/lon location. Measures distance and if the target is reached.
        """
        timeout = 20
        condition_yaw(self.vehicle,self.vehicle.heading) 
        self.vehicle.simple_goto(target)
        start = time.time()    
        while self.vehicle.mode.name=="GUIDED":
            current = time.time() - start
            dTarget = sqrt(pow(target.lat-self.vehicle.location.global_frame.lat,2)+pow(target.lon-self.vehicle.location.global_frame.lon,2)+pow(target.alt-self.vehicle.location.global_frame.alt,2))
            print " -> T: %0.1f, Alt: %0.1f, ToGo: %0.2f" % (current, self.vehicle.location.global_frame.alt, dTarget)
            if abs(self.vehicle.location.global_relative_frame.alt-target.alt) < 0.05: 
                print "\n\tReached %0.1f m in %0.1f sec!\n" % (target.alt, current)
                break
            time.sleep(0.5)


    def go_to(self, target):
        """
        Function that makes the vehicle travel to an specific lat/lon location. Measures distance and if the target is reached.
        """
        timeout = 20
        min_distance = 0.000005 # Parameter to tune by experimenting
        self.vehicle.simple_goto(target)
        start = time.time()    
        while self.vehicle.mode.name=="GUIDED":
            current = time.time() - start
            dTarget = sqrt(pow(target.lat-self.vehicle.location.global_frame.lat,2)+pow(target.lon-self.vehicle.location.global_frame.lon,2)++pow(target.alt-self.vehicle.location.global_frame.alt,2))
            print " ->T:%0.1f, Target[%0.2f %0.2f %0.1f], Actual[%0.2f %0.2f %0.1f], ToGo:%0.6f" % (current, target.lat, target.lon, target.alt, self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt, dTarget)
            if dTarget<=min_distance:
                print "Reached target location"
                break;
            if current >= timeout:
                print "Timeout to reach location, last distance: %0.4f" % (dTarget)
                break;
            time.sleep(0.5)


    def set_ned_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush()

    def set_velocity_body(self, vx, vy, vz):
        """ Remember: vz is positive downward!!!
        http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        
        Bitmask to indicate which dimensions should be ignored by the vehicle 
        (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
        none of the setpoint dimensions should be ignored). Mapping: 
        bit 1: x,  bit 2: y,  bit 3: z, 
        bit 4: vx, bit 5: vy, bit 6: vz, 
        bit 7: ax, bit 8: ay, bit 9:
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111, #-- BITMASK -> Consider only the velocities
                0, 0, 0,        #-- POSITION
                vx, vy, vz,     #-- VELOCITY
                0, 0, 0,        #-- ACCELERATIONS
                0, 0)
        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush()
        
    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def goto_position_target_local_ned(self, north, east, down):
        """ 
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        location in the North, East, Down frame.
        It is important to remember that in this frame, positive altitudes are entered as negative 
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    #def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    #    """
    #    Move vehicle in direction based on specified velocity vectors and
    #    for the specified duration.
    #    """
    #    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #        0,       # time_boot_ms (not used)
    #        0, 0,    # target system, target component
    #        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    #        0b0000111111000111, # type_mask (only speeds enabled)
    #        0, 0, 0, # x, y, z positions (not used)
    #        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
    #        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    #        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    #
    #    # send command to vehicle on 1 Hz cycle
    #    for x in range(0,duration):
    #        vehicle.send_mavlink(msg)
    #        time.sleep(duration)

    def set_global_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        #for x in range(0,duration):
        self.vehicle.send_mavlink(msg)
        #time.sleep(1)    

    def goto_position_target_global_int(self,aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)



    def goto(self,dNorth, dEast, gotoFunction=self.vehicle.simple_goto):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
        the target position. This allows it to be called with different position-setting commands. 
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        gotoFunction(targetLocation)
        
        #print "DEBUG: targetLocation: %s" % targetLocation
        #print "DEBUG: targetLocation: %s" % targetDistance

        while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            #print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance=get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(2)


    def set_roi(self,location):
        """
        Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
        specified region of interest (LocationGlobal).
        The vehicle may also turn to face the ROI.

        For more information see: 
        http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
        """
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
            0, #confirmation
            0, 0, 0, 0, #params 1-4
            location.lat,
            location.lon,
            location.alt
            )
        # send command to vehicle
        self.vehicle.send_mavlink(msg)    


    

    def set_attitude(self,roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5):
        """
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).
        """
        
        """
        The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
        so you must use quaternion to control the pitch and roll for those vehicles.
        """
        
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000, # Type mask: bit 1 is LSB
            to_quaternion(roll_angle, pitch_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

        #start = time.time()
        #while time.time() - start < duration:
         #   self.vehicle.send_mavlink(msg)
         #    time.sleep(0.1)


    def set_attitude_duration(self,roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0.0):
            """
            Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
            with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
            velocity persists until it is canceled. The code below should work on either version
            (sending the message multiple times does not cause problems).
            """
            
            """
            The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
            so you must use quaternion to control the pitch and roll for those vehicles.
            """
            
            # Thrust >  0.5: Ascend
            # Thrust == 0.5: Hold the altitude
            # Thrust <  0.5: Descend
            msg = self.vehicle.message_factory.set_attitude_target_encode(
                0, # time_boot_ms
                1, # Target system
                1, # Target component
                0b00000000, # Type mask: bit 1 is LSB
                to_quaternion(roll_angle, pitch_angle), # Quaternion
                0, # Body roll rate in radian
                0, # Body pitch rate in radian
                math.radians(yaw_rate), # Body yaw rate in radian
                thrust  # Thrust
            )
            self.vehicle.send_mavlink(msg)

            start = time.time()
            while time.time() - start < duration:
                self.vehicle.send_mavlink(msg)
                time.sleep(0.1)

    def move_servo(self, port, value):
        """
        Function that moves a servo from a specified port and value
        port  -> port where the servo is attached
        value -> servo ms value, from 1000 - 2000
        """
        msg = self.vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, port, value, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)