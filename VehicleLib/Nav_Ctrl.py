import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil


from math import pi; #Definition of pi from math library.

# WGS-84
R0 = 6378137.0;      #Earth's Radius at the equator, semi-major.
Rp = 6356752.314245;     #Earth's Radius at the pole, semi-minor.
f = 1/298.257223563;     #Earth's Flattening
ecc = 0.0818191908425;   #Earth's eccentricity
omega_E = 7.292115e-5;   #15.041 deg/hr, includes revolution about sun.
g0 = 9.780373;       #Gravity
gc1 = 0.0052891;     #Local Gravity Computation
gc2 = 0.0000059;     #Local Gravity Computation
g = 9.81             # m/s2 - gravity
# Unit Conversions
d2r = pi/180;            #Degree to Radian
r2d = 1/d2r;             #Radian to Degree
f2m=0.3048;              #Feet to Meter
kts2ms = 0.514444444;    #Knots to m/s

# COLUMN ID
fg_tcol = 0;    fg_latcol = 1;  fg_loncol = 2;  fg_altcol = 3;
fg_TAScol = 4;  fg_GScol = 5;
fg_phicol = 6;  fg_thecol = 7;  fg_psicol = 8;
fg_tracol = 9;  fg_fpacol = 10;
fg_pcol = 11;   fg_qcol = 12;   fg_rcol = 13;
fg_ucol = 14;   fg_vcol = 15;   fg_wcol = 16;
fg_VNcol = 17;  fg_VEcol = 18;  fg_VDcol = 19;

airdat_tcol = 0;    airdat_latcol = 1;  airdat_loncol = 2;  airdat_altcol = 3;
airdat_TAScol = 4;  airdat_VNcol = 5;   airdat_VEcol = 6;   airdat_VDcol = 7;
airdat_tracol = 8;  airdat_fpacol = 9;
airdat_ucol = 10;   airdat_vcol = 11;   airdat_wcol = 12;

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------

#------mission functions-------------
def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    print "Downloading mission"
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
        
    return n_WP, missionList
    

def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
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

def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """ 

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print(" Upload new commands to vehicle")
    cmds.upload()


#--------Navigation functions--------------------
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """ 
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;
def _get_location_metres(self, original_location, dNorth, dEast, is_global=False):
        """
        Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location has the same `alt and `is_relative` values
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        
        if is_global:
            return LocationGlobal(newlat, newlon,original_location.alt)    
        else:
            return LocationGlobalRelative(newlat, newlon,original_location.alt)   
def get_target_from_bearing(self, original_location, ang, dist, altitude=None):
        """ Create a TGT request packet located at a bearing and distance from the original point
        
        Inputs:
            ang     - [rad] Angle respect to North (clockwise) 
            dist    - [m]   Distance from the actual location
            altitude- [m]
        Returns:
            location - Dronekit compatible
        """
        
        if altitude is None: altitude = original_location.alt
        
        # print '---------------------- simulate_target_packet'
        dNorth  = dist*math.cos(ang)
        dEast   = dist*math.sin(ang)
        # print "Based on the actual heading of %.0f, the relative target's coordinates are %.1f m North, %.1f m East" % (math.degrees(ang), dNorth, dEast) 
        
        #-- Get the Lat and Lon
        tgt     = self._get_location_metres(original_location, dNorth, dEast)
        
        tgt.alt = altitude
        # print "Obtained the following target", tgt.lat, tgt.lon, tgt.alt

        return tgt      

def ground_course_2_location(self, angle_deg, altitude=None):
        """ Creates a target to aim to in order to follow the ground course
        Input:
            angle_deg   - target ground course
            altitude    - target altitude (default the current)
        
        """
        tgt = self.get_target_from_bearing(original_location=self.location_current, 
                                             ang=math.radians(angle_deg), 
                                             dist=5000,
                                             altitude=altitude)
        return(tgt)


def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]
######################## ATTITUDE TOOLBOX ##############################
# Functino att2body

def euler2tilt_errors(yaw, pitch, roll, input_units='rad', rotation_sequence='321'):
    """
    Returns a transformation matrix to convert from Euler Angle error states
    (delta_roll, delta_pitch, delta_yaw) to tilt angle (about NED frame) error 
    states (e_north, e_east, e_down).  Mathematically:
    
    [tilt errors] = Omega_T * [Euler angle errors]
 
    Parameters
    ----------   
    yaw   : yaw angle, units of input_units.
    pitch : pitch angle, units of input_units.
    roll  : roll angle , units of input_units.
    input_units: units for input angles {'rad', 'deg'}, optional.
    rotationSequence: assumed rotation sequence {'321', others can be 
                                                implemented in the future}.
    Returns
    -------
    Omega_T: 3x3 transformation matrix (numpy matrix data type).  This can be
               used to convert Euler angle error states to tilt angle errors.
    
    Note
    ----
    This is NOT a proper transformation matrix (i.e it is not normal + unitary).
    
    Reference
    ---------
    Equation 2.80, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    """
    # Apply necessary unit transformations.
    if input_units == 'rad':
        pass
    elif input_units == 'deg':
        yaw, pitch, roll = np.radians([yaw, pitch, roll])

    # Build transformation matrix Rnav2body.
    s_p, c_p = sin(pitch), cos(pitch)
    s_y, c_y = sin(yaw)  , cos(yaw)
    
    if rotation_sequence == '321':
        # The assumed Euler Angle sequence is '321'
        Omega_T = np.matrix([
                    [c_p*c_y, -s_y, 0],
                    [c_p*s_y,  c_y, 0],
                    [   -s_p,    0, 1]], dtype=float)

    else: 
        # No other rotation sequence is currently implemented
        print('WARNING (euler2tilt_errors): assumed rotation_sequence is unavailable.')
        print('                     NaN returned.')
        Omega_T = np.nan
    
    return Omega_T
    
    

def angle2dcm(yaw, pitch, roll, input_units='rad', rotation_sequence='321'):
    """
    Returns a transformation matrix (aka direction cosine matrix or DCM) which 
    transforms from navigation to body frame.  Other names commonly used, 
    besides DCM, are `Cbody2nav` or `Rbody2nav`.  The rotation sequence 
    specifies the order of rotations when going from navigation-frame to 
    body-frame.  The default is '321' (i.e Yaw -> Pitch -> Roll).

    Parameters
    ----------
    yaw   : yaw angle, units of input_units.
    pitch : pitch angle, units of input_units.
    roll  : roll angle , units of input_units.
    input_units: units for input angles {'rad', 'deg'}, optional.
    rotationSequence: assumed rotation sequence {'321', others can be 
                                                implemented in the future}.

    Returns
    -------
    Rnav2body: 3x3 transformation matrix (numpy matrix data type).  This can be
               used to convert from navigation-frame (e.g NED) to body frame.
        
    Notes
    -----
    Since Rnav2body is a proper transformation matrix, the inverse
    transformation is simply the transpose.  Hence, to go from body->nav,
    simply use: Rbody2nav = Rnav2body.T

    Examples:
    ---------
    >>> import numpy as np
    >>> from nav import angle2dcm
    >>> g_ned = np.matrix([[0, 0, 9.8]]).T # gravity vector in NED frame
    >>> yaw, pitch, roll = np.deg2rad([90, 15, 0]) # vehicle orientation
    >>> g_body = Rnav2body * g_ned
    >>> g_body
    matrix([[-2.53642664],
            [ 0.        ],
            [ 9.4660731 ]])
            
    >>> g_ned_check = Rnav2body.T * g_body 
    >>> np.linalg.norm(g_ned_check - g_ned) < 1e-10 # should match g_ned
    True

    Reference
    ---------
    [1] Equation 2.4, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] eul2Cbn.m function (note, this function gives body->nav) at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz
    """
    # Apply necessary unit transformations.
    if input_units == 'rad':
        pass
    elif input_units == 'deg':
        yaw, pitch, roll = np.radians([yaw, pitch, roll])

    # Build transformation matrix Rnav2body.
    s_r, c_r = sin(roll) , cos(roll)
    s_p, c_p = sin(pitch), cos(pitch)
    s_y, c_y = sin(yaw)  , cos(yaw)

    if rotation_sequence == '321':
        # This is equivalent to Rnav2body = R(roll) * R(pitch) * R(yaw)
        # where R() is the single axis rotation matrix.  We implement
        # the expanded form for improved efficiency.
        Rnav2body = np.matrix([
                [c_y*c_p               ,  s_y*c_p              , -s_p    ],
                [-s_y*c_r + c_y*s_p*s_r,  c_y*c_r + s_y*s_p*s_r,  c_p*s_r],
                [ s_y*s_r + c_y*s_p*c_r, -c_y*s_r + s_y*s_p*c_r,  c_p*c_r]])

    else: 
        # No other rotation sequence is currently implemented
        print('WARNING (angle2dcm): requested rotation_sequence is unavailable.')
        print('                     NaN returned.')
        Rnav2body = np.nan
    
    return Rnav2body

def Rbody2nav_to_angle(R, output_units='rad', rotation_sequence='321'):
    """
    Returns a Euler angles derived from transformation matrix R (body->nav).
    This transformation matrix is the transpose of the DCM.
    The rotation sequence specifies the order of rotations which were assumed
    when forming the DCM (R.T). The default is '321' (i.e Yaw -> Pitch -> Roll).

    Parameters
    ----------
    R : 3x3 numpy array or matrix
        Transformation matrix from body -> nav (transpose of DCM)
    output_units : {'rad', 'deg'}, optional
                   Units for output Euler angles.
    rotationSequence : {'321', others can be implemented in the future}
                       Assumed rotation sequence for the supplied DCM (R.T).

    Returns
    -------
    yaw   : yaw angle, units of output_units.
    pitch : pitch angle, units of output_units.
    roll  : roll angle , units of output_units.
    
    
    Reference
    ---------
    [1] Equation 2.45-47, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] Cbn2eul.m function at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz        
    """
    yaw = np.arctan2(R[1,0], R[0,0])
    #pitch = -np.arctan(R[2,0] / np.sqrt(1.-R[2,0]**2)) # Farrel eqn 2.45
    pitch =  -np.arcsin(R[2,0]) #  this is simpler
    roll  = np.arctan2(R[2,1], R[2,2] )
    
    # Apply necessary unit transformations.
    if output_units == 'rad':
        pass
    elif output_units == 'deg':
        yaw, pitch, roll = np.degrees([yaw, pitch, roll])
        
    return yaw, pitch, roll
    

def omega2rates(pitch, roll, input_units='rad', euler_angles_order='roll_pitch_yaw'):
    """
    This function is used to create the transformation matrix to go from:
        [p, q, r] --> [roll_rate, pitch_rate, yaw_rate]
    where pqr are xyz body rotation-rate measurements expressed in body frame.
    Yaw, pitch, and roll are the Euler angles.  We assume the Euler angles are
    3-2-1 (i.e Yaw -> Pitch -> Roll) transformations that go from navigation-
    frame to body-frame.

    Parameters
    ----------
    pitch : pitch angle, units of input_units.
    roll  : roll angle , units of input_units.
    input_units: units for input angles {'rad', 'deg'}, optional.
    euler_angles_order: assumed order of Euler Angles attitude state vector
                        {'roll_pitch_yaw', 'yaw_pitch_roll'} (see ``Notes``).

    Returns
    -------
    R: transformation matrix, from xyz body-rate to Euler angle-rates
       numpy array 3x3 (Note: the return variable is an ARRAY, not a matrix)
        
    Notes
    -----
    Since the returned transformation matrix is used to transform one vector
    to another, the assumed attitude variables order matters.  
    The ``euler_angles_order`` parameter can be used to specify the assumed order.

    The difference is demonstrated by example:
        # By default euler_angles_order='roll_pitch_yaw'
        R = omega2rates(pitch, roll) 
        [ roll_rate]         [omega_x]
        [pitch_rate] = dot(R,[omega_y])
        [  yaw_rate]         [omega_z]

        # Now assume our attitude state is [yaw, pitch, roll].T
        R = omega2rates(pitch, roll, euler_angles_order='yaw_pitch_roll') 
        [ yaw_rate]          [omega_x]
        [pitch_rate] = dot(R,[omega_y])
        [ roll_rate]         [omega_z]  

    Reference
    ---------
    [1] Equation 2.74, Aided Navigation: GPS with High Rate Sensors, Jay A. Farrel 2008
    [2] omega2rates.m function at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz
    """
    # Apply necessary unit transformations.
    if input_units == 'rad':
        pitch_rad, roll_rad = pitch, roll
    elif input_units == 'deg':
        pitch_rad, roll_rad = np.radians([pitch, roll])

    # Build transformation matrix.
    s_r, c_r = sin( roll_rad), cos( roll_rad)
    s_p, c_p = sin(pitch_rad), cos(pitch_rad)
    
    # Check for singularities (i.e. pitch near 90 degrees)
    singular_tol = 1e-2; # flags anything between [90 +/- .5 deg]
    if abs(c_p) < singular_tol:
        print('WARNING (omega2rates): Operating near pitch = 90 deg singularity.  NaN returned. ')
        return np.nan

    if euler_angles_order == 'roll_pitch_yaw':
        R = np.array(
           [[  1, s_r*s_p/c_p,  c_r*s_p/c_p],
            [  0, c_r        , -s_r        ],
            [  0, s_r/c_p    ,  c_r/c_p    ]], dtype=float)
    elif euler_angles_order == 'yaw_pitch_roll':
        R = np.array(
           [[  0, s_r/c_p    ,  c_r/c_p    ],
            [  0, c_r        , -s_r        ],
            [  1, s_r*s_p/c_p,  c_r*s_p/c_p]], dtype=float)

    return R


###################### MISCELLANEOUS FUNCTION ##########################
def sk(w):
    """
    This function gives a skew symmetric matrix from a given vector w
    Input:
        w: A vector, 3x1 or 1x3
    Output:
        C: Skew Symmetric Matrix, 3x3
    Programmer:    Adhika Lie
    Created:         May 09, 2011
    Last Modified: May 10, 2011
    
    May 10 - Fix syntax error
    """
    # TODO: this will not work if w is an matrix!
    C=numpy.matrix([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]);
    
    return C;


def wrap_pi(angles_rad):
    """
    Wrap radian angles to [-PI, +PI].
    
    Parameters
    ----------
    angle: input angles (list or np.array, units: radians)
    
    Returns
    -------
    wrapped_angle: angles wrapped to [-PI, +PI] (np.array, units: radians)
    
    Based on Adhika Lie wrapToPi.m 1/16/13 function.
    
    Note
    ----
    Care is needed at boundries.  For example, current function will wrap both
    +/- PI to -PI.  Possible future implementations may be improved to better
    handle boundry cases. 
    """
    return np.mod(angles_rad+np.pi, 2.0 * np.pi) - np.pi

""" Differentiate given value of position to calculate velocity, also has an added Low Pass Filter """
class velocity:
    def __init__(self,bandwidth,dt):
        self.filter_bandwidth=bandwidth
        self.dt=dt
        self.vel=0.0
        self.pos_past=0.0
        self.filter=0.0
        self.filter_past=0.0
    def get_velocity(self,current_value):
        self.vel = ( current_value - self.pos_past ) / self.dt
        self.pos_past = current_value
        self.filter = self.filter_past + self.dt * ( self.filter_bandwidth * ( self.vel - self.filter_past ) )
        self.filter_past = self.filter
        return self.vel,self.filter

""" Function that returns the x and y coordinates of a circle with specific radius and speed, to be used for trajectories """
def circle_trajectory(r, w, step):
    x = round(r*cos(w*step),2)
    y = round(r*sin(w*step),2)
    return x,y


""" Function that returns the x and y coordinates of an infinity figure, figure of eight, also known as the lemniscate of Bernoulli. """
def infinity_trajectory(a, b, w, step):
    y = round( a*sqrt(2)*cos(w*step) / (sin(w*step)*sin(w*step) + 1) ,2)
    x = round( b*sqrt(2)*cos(w*step)*sin(w*step) / (sin(w*step)*sin(w*step) + 1) ,2)
    return x,y


""" Function to map a value to another """
def toPWM(value, option, angle_limit=50):
    iMin = -angle_limit
    iMax = angle_limit
    if option == 1: # Normal
      oMin = 1000
      oMax = 2000
    elif option == -1: # Inverted
      oMin = 2000
      oMax = 1000
    return round((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin, 0)

""" Function to map a value to another """
def mapping(value,iMin,iMax,oMin,oMax):
    return ((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin)

""" Function to limit a value to specific range """
def limit(n, minn, maxn):
    return max(min(maxn, n), minn)

""" Kalman Filter Class """
class KalmanFilter(object):
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate



#-----control functions and filters------

""" Discrete PID control Class """
class PID:
    def __init__(self, P, I, D, filter_bandwidth, Derivator=0, Integrator=0, dt=0.01, Integrator_max=1.0, Integrator_min=-1.0):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.dt=dt
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.set_point=0.0
        self.error=0.0
        self.filter_bandwidth=filter_bandwidth
        self.filter=0.0
        self.filter_past=0.0

    def update(self, error):
        """
        Calculate PID output value for given reference input and feedback
        """

        #self.error = self.set_point - current_value
        self.error = error

        # Proportional term
        self.P_value = self.Kp * self.error

        # Filter
        self.filter = self.filter_past + self.dt * ( self.filter_bandwidth * ( self.error - self.filter_past ) )
        self.filter_past = self.filter

        # Derivative term
        self.D_value = self.Kd * (( self.filter - self.Derivator ) / self.dt )
        self.Derivator = self.filter
        #self.D_value = self.Kd * (( self.error - self.Derivator ) / self.dt )
        #self.Derivator = self.error

        # Integral term
        self.Integrator = self.Integrator + self.error * self.dt

        #if self.Integrator > self.Integrator_max:
        #    self.Integrator = self.Integrator_max
        #elif self.Integrator < self.Integrator_min:
        #    self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        #self.Integrator=0
        #self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

    def resetIntegrator(self):
        self.Integrator=0


""" Low Pass Filter """
class low_pass:
    def __init__(self,bandwidth,dt):
        self.filter_bandwidth=bandwidth
        self.dt=dt
        self.filter=0.0
        self.filter_past=0.0
    def update(self,current_value):
        self.filter = self.filter_past + self.dt * ( self.filter_bandwidth * ( current_value - self.filter_past ) )
        self.filter_past = self.filter
        return self.filter
