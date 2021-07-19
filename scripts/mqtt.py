#!/usr/bin/env python2

import requests, time, math
import json, time
import paho.mqtt.client as mqtt

import rospy
from tf.transformations import quaternion_from_euler
import tf

from sensor_msgs.msg import Imu, NavSatFix 

#url = 'http://192.168.0.66/'
url = 'http://10.42.0.185/'
mqtt_client_name = "ROS-mqtt-client"
mqtt_server_ip =  "192.168.1.2"
mqtt_topic =  "phone/data"

headers = {
    'Accept': '*/*',
    'Accept-Language': 'en-US,en;q=0.5',
    'Connection': 'close',
    'Referer': url
}

params_start  = (
    ('cmd', 'start'),
)

params_stop  = (
    ('cmd', 'stop'),
)

params_clear  = (
    ('cmd', 'clear'),
)

def compute_heading(x, y):
    return math.atan2(x, y) * 180.0/math.pi

def on_connect(client, userdata, flags, rc):
    mqtt_client.subscribe(mqtt_topic)

def on_disconnect(client, userdata,rc=0):
    print "Disconnecting"
#    client.loop_stop()
    client.disconnect()

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'base_link'

    imu_msg.linear_acceleration.x = data['lin_accY']
    imu_msg.linear_acceleration.y = -1*data['lin_accX']
    imu_msg.linear_acceleration.z = data['lin_accZ']
    imu_msg.angular_velocity.x = data['gyroY']
    imu_msg.angular_velocity.y = -1*data['gyroX']
    imu_msg.angular_velocity.z = data['gyroZ']

#    azimuth_xy = compute_heading(data['magX'], data['magY'])
#    azimuth_yz = compute_heading(data['magY'], data['magZ'])
#    azimuth_zx = compute_heading(data['magZ'], data['magX'])
    yaw = compute_heading(data['magX'], data['magY'])
    pitch = compute_heading(data['magY'], data['magZ'])
    roll = compute_heading(data['magZ'], data['magX'])
    #print "yaw: ", yaw


    #heading_quat = quaternion_from_euler(azimuth_yz, azimuth_zx, azimuth_xy)
    heading_quat = quaternion_from_euler(roll, pitch, yaw)

    imu_msg.orientation.x = heading_quat[0]
    imu_msg.orientation.y = heading_quat[1]
    imu_msg.orientation.z = heading_quat[2]
    imu_msg.orientation.w = heading_quat[3]

    imu_pub.publish(imu_msg)

    gps_msg = NavSatFix()
    if data['gps_time'] != None:
        gps_msg.header.stamp = rospy.Time(data['gps_time'])
    gps_msg.header.frame_id = 'map'
    gps_msg.latitude = data['gpsLat']
    gps_msg.longitude = data['gpsLon']
    gps_msg.altitude = data['gpsZ']

    gps_pub.publish(gps_msg)

    #br = tf.TransformBroadcaster()
    #br.sendTransform((1.0, 2.0, 5.0),
    #                 tf.transformations.quaternion_from_euler(roll, pitch, yaw),
    #                 rospy.Time.now(),
    #                 "base_link",
    #                 "map")
    #print "Sent TF transform"


    
if __name__ == '__main__':

#    _ = requests.get(url+'control', headers=headers, params=params_clear)
#        _ = requests.get(url+'control', headers=headers, params=params_start)
    rospy.init_node('phyphox_imu')

    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)


    mqtt_client = mqtt.Client(mqtt_client_name)

    mqtt_client.on_message = on_message
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect

    mqtt_client.connect(host=mqtt_server_ip, keepalive=60)

    #try:
    mqtt_client.loop_forever()
#    mqtt_client.loop_start()
#    mqtt_client.loop_stop()

    #except KeyboardInterrupt:
    #    print("Node interrupted")
#
#    finally:
#        _ = requests.get(url+'control', headers=headers, params=params_stop)
#        _ = requests.get(url+'control', headers=headers, params=params_clear)
