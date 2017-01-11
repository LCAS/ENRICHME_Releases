#!/usr/bin/env python


from subprocess import call
import rospy
from ros_alchemy.srv import InferenceSrv, InferenceSrvRequest, InferenceSrvResponse
from ros_alchemy.msg import Inference
import os
import inspect


# Node class.
class alchemy_server():


    def performInference(self,inferenceRequest):
        ans=InferenceSrvResponse()

        model= inferenceRequest.model
        evidenceSTR=inferenceRequest.evidence
        queries=','.join(inferenceRequest.queries)

        rospy.logdebug("Received service request.")
        rospy.logdebug("Model: %s", model)
        rospy.logdebug("Queries: %s",queries)
        rospy.logdebug("Evidences: %s", evidenceSTR)
        #####################################################################################
        #  inspect.getfile(inspect.currentframe())  # script filename (usually with path)
        basePath=os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))  # script directory
        alchemyDir=basePath+"/../alchemy"

        network = ''.join([basePath,"/../networks/",model,"/network-trained.mln"])
        rospy.logdebug("Usin network: %s", network)

        evidenceDB = "evidence2.db"
        resultFile ='out.txt'
        outputFile = 'screenOut.txt'
        f1= open(evidenceDB, 'w')

        for element in evidenceSTR:
            f1.write(element+'\n')
        f1.close()

        command = [
            alchemyDir + '/infer']

        data=[
            '-i', network,  # Comma-separated input .mln files
            '-r', resultFile,  # The probability estimates are written to this file.
            '-e', evidenceDB,  # The probability estimates are written to this file.
            '-q', queries]


        methodMCSAT=[
            '-ms', '1',              # Run inference using MC-SAT and return probabilities for all query atoms
            '-numSolutions', '10',  # Return nth SAT solution in SampleSaT
            '-saRatio', '0',        # Ratio of sim. annealing steps mixed with WalkSAT in MC-SAT
            '-saTemperature', '10'  # Temperature (/100) for sim. annealing step in SampleSat
        ]

        methodGIBS=[
        '-p','true',                # Run inference using MCMC (Gibbs sampling) and return probabilities for all query atoms.
        '-numChains', '50'          # [10] Number of MCMC chains for Gibbs sampling (there must be at least 2).
        ]


        command=command+methodGIBS+data

        f2 = open(outputFile, 'a')
        #FNULL = open(os.devnull, 'w')
        #out=call(command, stdout=FNULL, stderr=FNULL)
        out = call(command, stdout=f2, stderr=f2)
        f2.close()

        rospy.logdebug("Calling Alchemy...............................................................................")

        f2 = open(outputFile, 'r')
        for line in f2:
            rospy.logdebug(line)
            #f2.write(line+'\n')
        f2.close()

        f = open(resultFile, 'r')
        ans.results=[]
        rospy.logdebug("")
        rospy.logdebug("From evidence:")
        for element in evidenceSTR:
            rospy.logdebug(element)
        rospy.logdebug("")
        rospy.logdebug("Inference result:")
        totalProb=0
        for line in f:
            tokens=line.split()
            activity=tokens[0].split('(')[1][0:-1]
            probability = float(tokens[1])*100

            inf=Inference()
            inf.functionName=activity
            inf.probability=probability
            totalProb+=inf.probability
            ans.results.append(inf)
            #rospy.logdebug("Prob(%s)=%2.2f %%" % (activity,probability))
        rospy.logdebug("")

        os.remove(evidenceDB)
        os.remove(resultFile)
        os.remove(outputFile)
            
        for inf in ans.results:
            inf.probability=100.0*inf.probability/totalProb
            rospy.logdebug("Prob(%s)=%2.2f %%" % (inf.functionName,inf.probability))
        return ans

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rospy.loginfo("Advertising MLN inference service ")
        self.s=rospy.Service('probcog_infer', InferenceSrv , self.performInference)
        rospy.loginfo("Ready to accept queries ")
        rospy.spin()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ros_alchemy_server' ,log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        a_s = alchemy_server()
    except rospy.ROSInterruptException: pass
