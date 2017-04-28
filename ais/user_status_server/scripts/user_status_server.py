#!/usr/bin/env python

"""
ROS Service for user status monitoring.
  Changelog: 
   - It uses mongoDB to check where user is.
   - Generates proper SOAP message to caregiver
   
   TODO pending changes in UserStatusServer
   - Generate periodic  user status request
   - Remove hard coded constants: 
            physiological info topic "healthMetrics"
            graphname "ais/user_status_server"
            service name "user_status"

"""


from user_status_server.srv import *
from user_status_server.msg import *
from enrichme_msgs.srv import *
from ncp_event_notifier.srv import *
from ncp_event_notifier.msg import *
from optris_reader.msg import Biometrics
import rospy



class user_status_server():



    # service handler
    def service_request(self,req):
        '''
        :param req: empty.
        :return: service response. See srv for structure
        '''

        response=self.locateUser()

        if (response.userStatus == "abnormal"):
            self.send_NCP_event()
        return response

    def locateUser(self):
        '''
        Internal logic behind service call. Decoupled from srv call, so it can be also called periodically
        :return:
        '''
        rospy.loginfo('User status request received')
        
        # location probabilities are sorted and given by location mln through topic
        # now we call HRI for updates on user location and status
        # providing it locationsProb 
        ReachPersonRequest.locations=self.mln_locs
        ReachPersonRequest.confidence=self.mln_probs
        rospy.loginfo("Asking robot to reach person: ")
        self.reachPersonSrvCall(ReachPersonRequest, ReachPersonResponse)
        
        hriUserLocation =ReachPersonResponse.location
        userStatus      =ReachPersonResponse.status
        rospy.loginfo("Robot says it's at "+hriUserLocation+" and status is "+userStatus)
        
        # hopefully by now healthmetrics should be updated...
        sep = ","
        seq = (self.healthMetrics.Temperature, self.healthMetrics.HearthRate, self.healthMetrics.RespirationRate)

        response = UserStatusResponse()
        response.userLocation=hriUserLocation      #confirmed user location
        response.userStatus=sep.join(seq)     #User status description
        return response

    def send_NCP_event(self):
        # Post event
        
        verb = 'POST'
        payload = NCP_Event()

        payload.ID = '74bafd06-d500-4319-9ecc-8524769bafbd'  # overriden!
        self.targetCD = str(rospy.get_param('/targetCD', '123456'))
        payload.TypeCD = 'UCDO'
        payload.Begin = rospy.Time.now()
        payload.End = rospy.Time.now()  # overriden ... i hope
        payload.Shown = rospy.Time.now()  # overriden ... i hope
        # payload.Conclusion
        # payload.Shown
        # payload.Disable
        payload.srcDomain = 'ULINAAL'
        payload.srcId='house01'
        # payload.Ext=''

        rospy.loginfo('NCP event ready')

        #while not rospy.is_shutdown():
        # Sleep before publishing new messages
        rospy.sleep(2)

        rospy.loginfo("Doing a %s operation", verb)
        # rospy.loginfo("With ID (%s)", payload.ID)

        try:
            resp1 = self.ens(verb, payload)
            rospy.loginfo("Response is %s", resp1.feedback)
            self.pub.publish(resp1.response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        
    def healthMetrics_callback(self,data):
        rospy.loginfo('health metrics data received')
        self.healthMetrics=data
        
    def mln_location_callback(self,data):
        rospy.loginfo('location data received')
        self.mln_rawData=data
        dataSpl = data.split(',')
        self.mln_locs=dataSpl[::2]
        self.mln_probs=dataSpl[1::2]
        self.mln_probs=[float(i) for i in self.mln_probs]


   # NOT NECESSARY. MLN ALREADY PUBLISHES USER LOCATION INFO PERIODICALLY.
   # periodic calls to user_status_server
   #  def periodicCall(self,event):
   #      response = self.locateUser()
   #
   #      if (response.userStatus == "abnormal"):
   #          self.send_NCP_event()
   #
   #      self.pub_period.publish()


    # class constructor.
    def __init__(self):
        rospy.init_node('user_status_node', log_level=rospy.DEBUG)

        # Get parameters
        # TODO: add login as a new server action
        self.userName = 'manuel.carmona'
        self.myPass = 'Prova!2016'
        self.baseURL = 'https://secure.tesanonline.it/pac/api'
        self.periodicUpdatesTime=rospy.Duration(rospy.get_param('periodicUpdatesTime',60))
        self.periodicTopic=rospy.get_param('periodicTopic','user_status_requests')


        # subscribe to biostatistics topic
        rospy.loginfo('Subscribing to health metrics topic')
        rospy.Subscriber("biometrics", Biometrics, self.healthMetrics_callback)

        # subscribe to mln location topic: act_rect_mln_node with location mln  
        rospy.loginfo('Subscribing to mln location topic')
        rospy.Subscriber("mln_location", std_msgs.msg.String, self.mln_location_callback)

        # client to ReachPerson service
        rospy.loginfo('Waiting for ReachPerson service to be active')
        rospy.wait_for_service('reachPerson')
        rospy.loginfo('Registering at reachPerson Service')        
        self.reachPersonSrvCall = rospy.ServiceProxy('reachPerson', ReachPerson)

        # client to ncp service
        rospy.loginfo('Waiting for NCP Event notifier server to be active')
        rospy.wait_for_service('ncp_event_operation')
        self.ens = rospy.ServiceProxy('ncp_event_operation', NCP_Event_operation)

        # advertise user status service
        s = rospy.Service('UserStatus', UserStatus, self.service_request)
        rospy.loginfo('User status server now active')

        # NOT NECESSARY. MLN ALREADY PUBLISHES USER LOCATION INFO PERIODICALLY.
        # create publisher for periodic updates
        #self.pub_period = rospy.Publisher(self.periodTopic, ReachPersonResponse, queue_size=10)
        # periodic publishing
        #rospy.Timer(self.periodicUpdatesTime, self.periodicCall)


        # main loop, wait for requests...
        rospy.spin()

# Main function........................................................................................................
if __name__ == "__main__":
    user_status_server()
