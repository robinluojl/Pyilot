from __future__ import print_function

from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import json, urllib, math
import time

def position_messages_from_tlog(filename):
    """
    Given telemetry log, get a series of wpts approximating the previous flight
    """
    # Pull out just the global position msgs
    messages = []
    mlog = mavutil.mavlink_connection(filename)
    while True:
        try:
            m = mlog.recv_match(type=['GLOBAL_POSITION_INT'])
            if m is None:
                break
        except Exception:
            break
        # ignore we get where there is no fix:
        if m.lat == 0:
            continue
        messages.append(m)

    # Shrink the number of points for readability and to stay within autopilot memory limits. 
    # For coding simplicity we:
    #   - only keep points that are with 3 metres of the previous kept point.
    #   - only keep the first 100 points that meet the above criteria.
    num_points = len(messages)
    keep_point_distance=3 #metres
    kept_messages = []
    kept_messages.append(messages[0]) #Keep the first message
    pt1num=0
    pt2num=1
    while True:
        #Keep the last point. Only record 99 points.
        if pt2num==num_points-1 or len(kept_messages)==99:
            kept_messages.append(messages[pt2num])
            break
        pt1 = LocationGlobalRelative(messages[pt1num].lat/1.0e7,messages[pt1num].lon/1.0e7,0)
        pt2 = LocationGlobalRelative(messages[pt2num].lat/1.0e7,messages[pt2num].lon/1.0e7,0)
        distance_between_points = get_distance_metres(pt1,pt2)
        if distance_between_points > keep_point_distance:
            kept_messages.append(messages[pt2num])
            pt1num=pt2num
        pt2num=pt2num+1

    return kept_messages