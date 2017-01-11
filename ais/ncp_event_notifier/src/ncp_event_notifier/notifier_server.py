#!/usr/bin/env python

"""
ROS Service that allows to POST/GET NCP Events.
 See msg and srv for structure.
"""


from ncp_event_notifier.srv import *
from ncp_event_notifier.msg import *
import rospy
import requests
import json
import datetime
import types

#TODO ALLOW CLASSES TO BE LOADED FROM FILE
'''
Previous version
classes= {'AMBI': {'ADOR','AHAB'}, \
          'CLIN': {'UCDO','URCT','UMCD','UNCT','UHAB','URFO'},\
          'HELP': {'UPPB'},\
          'SOCI': {'DPSM','DRSM','UHLH','UMPC','URST','URTH','DHAB'},\
          'TECH': {'DIFC','DHLB','DIBR','DNIC','DOOC','DCFO'} }
'''
classes = {'AMBI': {'ADOR', 'AHAB'}, \
           'CLIN': {'UCDO', 'URCT', 'UCDU', 'UNCT', 'UHAB', 'URFO'}, \
           'HELP': {'UPPB'}, \
           'SOCI': {'DPSM', 'DRSM', 'UHLH', 'URTH'}, \
           'TECH': {'DIBR'}}

#TODO ALLOW EVENT TYPES TO BE LOADED FROM FILE
'''
Previous version
eventTypes = {'ADOR', 'AHAB', \
           'UCDO', 'URCT', 'UMCD', 'UNCT', 'UHAB', 'URFO', \
           'UPPB', \
           'DPSM', 'DRSM', 'UHLH', 'UMPC', 'URST', 'URTH', 'DHAB', \
           'DIFC', 'DHLB', 'DIBR', 'DNIC', 'DOOC', 'DCFO'}
'''
eventTypes = {  'ADOR', 'AHAB', \
           'UCDO', 'URCT', 'UCDU', 'UNCT', 'UHAB', 'URFO', \
           'UPPB', \
           'DPSM', 'DRSM', 'UHLH', 'URTH', \
           'DIBR'}


'''
DESCRIPTION:
AMBI
ADOR Ambient Data is Out-of-Range
AHAB Ambient Has Abnormal Behaviour

CLIN
UCDO User Clinical Data Out-of-range
UCDU User Clinical Data Unsafe
URCT User Refuses Clinical Task
UNCT User has Not done a Clinical Task
UHAB User Has Abnormal Behaviour
URFO User Requested Finding an Object

COGN
UECD User is Experiencing Cognitive Degradation
URST User Refuses Social Task

HELP
UPPB User Pressed Panic Button

SOCI
DPSM Device has been Put into Silent Mode
DRSM Device has been Removed from Silent Mode
UHLH User Has Left his Home
URTH User Returned To Home

TECH
DIBR Device Is BRoken
'''

class notifier_server():

    def postCallback(self,req):
        '''
        :param req: service request containing a post command
        :return: NCP response to post command. see srv for more details
        '''

        newNCPEvent = req.payload

        isOk = self.checkNCPEvent(newNCPEvent)

        if not isOk:
            response = self.createServiceResponse("Event is not well formed", newNCPEvent, False)
        else:
            postURL = self.baseURL + '/event/'
            jsonEvent = self.castToJson(newNCPEvent)
            r = requests.post(postURL, headers=self.myHeader, cookies=self.cuki, data=jsonEvent,verify=False)
            rospy.logdebug("Doing post with data (%s)", jsonEvent)
            if (r.status_code == requests.codes.ok)|(r.status_code == 201):
                newNCPEvent = self.castFromJson2NCPEvent(r.json())
                response = self.createServiceResponse(r.text, newNCPEvent, True)
            else:
                response = self.createServiceResponse(r.text, newNCPEvent, False)

        return response

    def getCallback(self,req):
        '''
        :param req: service request containing a get command
        :return: NCP response to get command. see srv for more details
        '''

        getURL = self.baseURL + '/event/' + req.payload.ID + '/'
        r = requests.get(getURL, headers=self.myHeader, cookies=self.cuki,verify=False)
        if (r.status_code == requests.codes.ok):
            newNCPEvent = self.castFromJson2NCPEvent(r.json())
            response = self.createServiceResponse(r.text, newNCPEvent, True)
        else:
            response = self.createServiceResponse(r.text, NCP_Event(), False)

        return response

    # TODO: put method is not tested. Is not implemented.
    def putCallback(self,req):
        '''
        :param req: put service request
        :return: NCP response to put request
        '''

        newEvent = req.payload

        isOk = self.checkNCPEvent(newEvent)

        if not isOk:
            response = self.createServiceResponse("Event is not well formed", newEvent, False)
        else:
            putURL = self.baseURL + '/event/' + newEvent.ID + '/'
            jsonEvent = self.castToJson(newEvent)
            r = requests.put(putURL, headers=self.myHeader, cookies=self.cuki, data=jsonEvent,verify=False)
            if (r.status_code == requests.codes.ok):
                newEvent = self.castFromJson2NCPEvent(r.json())
                response = self.createServiceResponse(r.text, newEvent, True)
            else:
                response = self.createServiceResponse(r.text, newEvent, False)
        return response

    def createServiceResponse(self,message, ncp_event, status):
        '''
        Builds a service response containing an event, message descriptor and status flag
        :param message: string describing response.
        :param ncp_event: response event from server / copy of sent event
        :param status: true if exchange was successful
        :return: service response containing these fields
        '''
        serviceResponse = NCP_Event_operationResponse()
        serviceResponse.response = ncp_event
        serviceResponse.wasOk = status
        serviceResponse.feedback = message

        if not status:
            rospy.logerr(serviceResponse.feedback)

        return serviceResponse

    def checkNCPEvent(self,ncpEvent):
        '''
        Checks if proposed event conforms to format:
        TargetCD string length is up to 15
        TypeCD string length is up to 4
        srcDomain string length is up to 7
        srcId string length is up to 64
        TypeCD is a registered event Type
        :param ncpEvent: event to be checked
        :return: True if event is properly formed
        '''
        isOk = True
        # check string sizes...
        if len(ncpEvent.TargetCD) > 15:
            isOk = False
        elif len(ncpEvent.TypeCD) > 4:
            isOk = False
        elif len(ncpEvent.srcDomain) > 7:
            isOk = False
        elif len(ncpEvent.srcId) > 64:
            isOk = False
        elif ncpEvent.TypeCD.upper() not in eventTypes:
            isOk = False

        return isOk

    def castToJson(self,ncpEvent):
        '''
        :param ncpEvent: ROS msg containing the NCP Event object
        :return: json object to be transmitted to server
        '''
        # cast event to a dict
        eventDict = {
            #                                                                            2016-05-30T09:00:00Z
            #'Begin': datetime.datetime.fromtimestamp(ncpEvent.Begin.to_sec()).strftime('%Y-%m-%dT%H:%M:%S'),
            'Begin': datetime.datetime.utcfromtimestamp(ncpEvent.Begin.to_sec()).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'TypeCD': ncpEvent.TypeCD,
            'End': datetime.datetime.utcfromtimestamp(ncpEvent.End.to_sec()).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'Name': ncpEvent.Name,
            'srcDomain': ncpEvent.srcDomain,
            'srcId': ncpEvent.srcId,
            'Level': ncpEvent.Level,
            'CentreCD': ncpEvent.CentreCD,
            'TargetCD': ncpEvent.TargetCD,
            'Shown': datetime.datetime.fromtimestamp(ncpEvent.Shown.to_sec()).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'Up': ncpEvent.Up,
            'EventName': ncpEvent.EventName,
            'Ext': ncpEvent.Ext,
            'Handling': ncpEvent.Handling,
            'Relevance': ncpEvent.Relevance,
            'Surname': ncpEvent.Surname,
            'ConclusionCD': ncpEvent.ConclusionCD,
            'ID': ncpEvent.ID,
            'Conclusion': ncpEvent.Conclusion
        }

        if ncpEvent.End.to_sec()==0.0:
            print 'deleting end Key'
            eventDict['End']=None
            #
            # del eventDict['End']


        # cast dict to json
        jsonAns = json.dumps(eventDict)
        return jsonAns

    def string2rosTime(self,timeString):
        '''
        :param timeString: String with format %Y-%m-%dT%H:%M:%SZ containing a date
        :return:rospy.Time object with that time or 0 if an error happens.
        '''
        rospytime = rospy.Time()

        try:
            if type(timeString) == 'string':
                dtTime = datetime.datetime.strptime(timeString, '%Y-%m-%dT%H:%M:%SZ')
                timestamp = (dtTime - datetime.datetime(1970, 1, 1)).total_seconds()
                rospytime = rospy.Time.from_sec(timestamp)
        except ValueError:
            pass

        return rospytime

    def castFromJson2NCPEvent(self,jsonData):
        '''
        :param jsonData: json document retrieved from NCP
        :return: NCP Event ROS message containing that data
        '''

        newNCPEvent = NCP_Event()
        newNCPEvent.Begin = self.string2rosTime(jsonData['Begin'])
        newNCPEvent.End = self.string2rosTime(jsonData['End'])
        newNCPEvent.Shown = self.string2rosTime(jsonData['Shown'])
        newNCPEvent.Level = int(jsonData['Level'])
        newNCPEvent.Relevance = int(jsonData['Relevance'])

        # not all fields are filled. Avoid casting exceptions
        if isinstance(jsonData['TypeCD'], types.UnicodeType):
            newNCPEvent.TypeCD = jsonData['TypeCD'].encode('ascii', 'ignore')
        if isinstance(jsonData['Name'], types.UnicodeType):
            newNCPEvent.Name = jsonData['Name'].encode('ascii', 'ignore')
        if isinstance(jsonData['srcDomain'], types.UnicodeType):
            newNCPEvent.srcDomain = jsonData['srcDomain'].encode('ascii', 'ignore')
        if isinstance(jsonData['srcId'], types.UnicodeType):
            newNCPEvent.srcId = jsonData['srcId'].encode('ascii', 'ignore')
        if isinstance(jsonData['CentreCD'], types.UnicodeType):
            newNCPEvent.CentreCD = jsonData['CentreCD'].encode('ascii', 'ignore')
        if isinstance(jsonData['TargetCD'], types.UnicodeType):
            newNCPEvent.TargetCD = jsonData['TargetCD'].encode('ascii', 'ignore')
        if isinstance(jsonData['Up'], types.UnicodeType):
            newNCPEvent.Up = jsonData['Up'].encode('ascii', 'ignore')
        if isinstance(jsonData['EventName'], types.UnicodeType):
            newNCPEvent.EventName = jsonData['EventName'].encode('ascii', 'ignore')
        if isinstance(jsonData['Ext'], types.UnicodeType):
            newNCPEvent.Ext = jsonData['Ext'].encode('ascii', 'ignore')
        if isinstance(jsonData['Handling'], types.UnicodeType):
            newNCPEvent.Handling = jsonData['Handling'].encode('ascii', 'ignore')
        if isinstance(jsonData['Surname'], types.UnicodeType):
            newNCPEvent.Surname = jsonData['Surname'].encode('ascii', 'ignore')
        if isinstance(jsonData['ConclusionCD'], types.UnicodeType):
            newNCPEvent.ConclusionCD = jsonData['ConclusionCD'].encode('ascii', 'ignore')
        if isinstance(jsonData['ID'], types.UnicodeType):
            newNCPEvent.ID = jsonData['ID'].encode('ascii', 'ignore')
        if isinstance(jsonData['Conclusion'], types.UnicodeType):
            newNCPEvent.Conclusion = jsonData['Conclusion'].encode('ascii', 'ignore')

        return newNCPEvent


    def service_request(self,req):
        '''
        :param req: ROS SERVICE REQUEST. See srv for structure.
        :return: service response. See srv for structure
        '''
        if req.verb.lower() == 'post':
            response = self.postCallback(req)
        elif req.verb.lower() == 'get':
            response = self.getCallback(req)
        elif req.verb.lower() == 'put':
            #response = self.createServiceResponse("Put is not active in REST interface \"" + req.verb + "\"", NCP_Event(),False)
            response = self.putCallback(req)
        else:
            response = self.createServiceResponse("Unknown verb \"" + req.verb + "\"", NCP_Event(), False)
        return response

    # class constructor.
    def __init__(self):
        rospy.init_node('ncp_event_notifier_server')#, log_level=rospy.DEBUG)

        # Get parameters
        # TODO: add login as a new server action
        self.userName = rospy.get_param('/NCPuserName','manuel.carmona')
        self.myPass = rospy.get_param('/NCPPass','Prova!2016')
        self.baseURL =rospy.get_param('/NCPbaseURL', 'https://secure.tesanonline.it/pac/api')
        # Login to NCP
        self.doLogin()

        s = rospy.Service('ncp_event_operation', NCP_Event_operation, self.service_request)
        rospy.loginfo('Event notifier server now active')

        # main loop, wait for requests...
        rospy.spin()

    # registers server as a event publisher
    # sets cookie for further transactions
    def doLogin(self):
        self.urlLogin = self.baseURL + '/account/login'

        self.myHeader = {'Content-Type': 'application/json', 'Accept': 'application/json'}
        self.loginData = {'user': self.userName, 'password': self.myPass}

        # Simple login (remember to get cookie for transactions)
        r = requests.post(self.urlLogin, data=json.dumps(self.loginData), headers=self.myHeader,verify=False)

        if (r.status_code != requests.codes.ok):
            raise rospy.exceptions.ROSInitException("Cant log into " + self.urlLogin)
        else:
            rospy.loginfo('Login successful: '+str(r.cookies))
        self.cuki = r.cookies

# Main function........................................................................................................
if __name__ == "__main__":
    notifier_server()
