#!/bin/python


'''
Code used to set all environmental ranges to an initial value.

'''


import requests
import json
import numpy as np

userName = 'manuel.carmona'
myPass = 'Prova!2016'
baseURL = 'https://secure.tesanonline.it/pac/api'
target  = '123456' #patient name

urlLogIn = baseURL + '/account/login'
urlLogout = baseURL + '/account'
urlCareplan = baseURL + '/careplan/'+target+'/ENRI'


prefix='ambient-'  # environmental parameters have this prefix on config file
magnitudeNamesList = ['CO',    'particleCount', 'VOC',    'temp',   'humidity', 'light']
comfortRangesList =  [[0, 60], [0, 20],         [85, 115], [18, 24], [30, 50],    [100, 250]]
safeRangesList =     [[0, 70], [0, 25],         [65, 135], [16, 25], [30, 60],    [float('-Inf'), float('Inf')]]
updatePeriodList =   [10,      10,              10,       30,       120,         300]

# login ...............................................................................................................

myHeader = {'Content-Type': 'application/json', 'Accept': 'application/json'}
loginData = {'user': userName, 'password': myPass}

# (remember to get cookie for transactions)
r = requests.post(urlLogIn, data=json.dumps(loginData), headers=myHeader, verify=False)

print('Login ........'+'\n')
print(str(r.status_code))
if (r.status_code != requests.codes.ok):
    print("Cant log into " + urlLogout)
    exit()
else:
    print('Login successful: '+str(r.cookies))
    cuki = r.cookies

# getting latest care plan  ...........................................................................................
print('')
print('Getting latest care plan ........')

r = requests.get(urlCareplan, headers=myHeader,verify=False,cookies=cuki)
print(str(r.status_code))
carePlan=r.json()
print(r.json())
revision=carePlan['care-plan']['revision']
print('On revision '+revision)
#exit()

# setting values .......................................................................................................
print('')
print('Deleting previous ambient ranges........')

nt=0
while(nt<len(carePlan['care-plan']['threshold'])):
    if prefix in carePlan['care-plan']['threshold'][nt]['name']:
        print "\t - Deleting " + carePlan['care-plan']['threshold'][nt]['name']
        del carePlan['care-plan']['threshold'][nt]
    else:
        nt+=1



print('')
print('Setting new ranges........')
for i in xrange(0,len(magnitudeNamesList)):
    t=dict()
    if np.isfinite(safeRangesList[i][0]):
        t['safe-min']=str(safeRangesList[i][0])
    if np.isfinite(safeRangesList[i][1]):
        t['safe-max']=str(safeRangesList[i][1])
    if np.isfinite(comfortRangesList[i][0]):
        t['comfort-min']=str(comfortRangesList[i][0])
    if np.isfinite(comfortRangesList[i][1]):
        t['comfort-max']=str(comfortRangesList[i][1])

    t['name']=prefix+magnitudeNamesList[i]
    carePlan['care-plan']['threshold'].append(t)

print('')
print('New ambient thresholds ........')
numThresholds=len(carePlan['care-plan']['threshold'])
for nt in xrange(0, numThresholds):
    if prefix in carePlan['care-plan']['threshold'][nt]['name']:
        print "\t - " + carePlan['care-plan']['threshold'][nt]['name']


#sending a modification to care plan....................................................................................
print('')
print('Submitting changes to care plan ........>')
carePlan['care-plan']['revision']=unicode(int(revision)+1)

r = requests.put(urlCareplan, headers=myHeader,data=json.dumps(carePlan),verify=False,cookies=cuki)
print(str(r.status_code))
print('')
print(r.json())

#login out............................................................................................................
print('')
print('Loging out ........')
r = requests.delete(urlLogout, verify=False, cookies=cuki)
print(str(r.status_code))
print(r.json())