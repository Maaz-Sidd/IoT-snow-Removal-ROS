#!/usr/bin/env python3

import rospy
from oauthlib.oauth2 import BackendApplicationClient
from requests_oauthlib import OAuth2Session
import iot_api_client as iot
from iot_api_client.rest import ApiException
from iot_api_client.configuration import Configuration
from std_msgs.msg import Int32 

def get_token():
    oauth_client = BackendApplicationClient(client_id="client_id")
    token_url = "https://api2.arduino.cc/iot/v1/clients/token"

    oauth = OAuth2Session(client=oauth_client)
    token = oauth.fetch_token(
        token_url=token_url,
        client_id="client_id",
        client_secret="Client_secret",
        include_client_id=True,
        audience="https://api2.arduino.cc/iot",
    )

    return token.get("access_token")

def interact_with_api(access_token):
    battery_pub = rospy.Publisher('/robot/battery', Int32, queue_size=1)

    client_config = Configuration(host="https://api2.arduino.cc/iot")
    client_config.access_token = access_token
    client = iot.ApiClient(client_config)
    thing_id = "thing_id"

    api = iot.PropertiesV2Api(client)

    try:
        resp = api.properties_v2_list(thing_id)
        rospy.loginfo(resp[1].last_value)
        battery_pub.publish(int(resp[1].last_value))
    except ApiException as e:
        rospy.logerr("Got an exception: {}".format(e))

def timer_callback(event):
    access_token = get_token()
    interact_with_api(access_token)

def main():
    rospy.init_node('iot_api_node', anonymous=True)

    rospy.Timer(rospy.Duration(10), timer_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
