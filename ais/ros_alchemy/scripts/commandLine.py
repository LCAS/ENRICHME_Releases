from subprocess import call
import os



#network    = "/home/mfernandezcarmona/Dropbox/ENRICHME/sciPro/intenv16/code/v5/alchemy/simple/act_LCAS-trained.mln"

network    = "/home/mfcarmona/Dropbox/ENRICHME/sciPro/intenv16/code/v5/alchemy/simple/act_LCAS-trained.mln"


query='currActivity'
evidenceDB = "evidence2.db"
resultFile ='out.txt'
f1= open(evidenceDB, 'w')


evidenceSTR = ['level(Low,Lounge_Multi_Presence)', 'level(Low,Kitchen_Multi_Presence)', 'level(Mid,Kitchen_Plug_Power)',
            'level(Mid,Toilet_Door_Contact)',
            'level(Mid,Entry_Multi_Contact)', 'level(Low,Entry_Multi_Presence)', 'level(Mid,Kitchen_Multi_Lux)',
            'level(Mid,Entry_Multi_Temp)',
            'level(Mid,Kitchen_Multi_Temp)', 'level(Mid,Lounge_Multi_Lux)', 'level(Mid,Lounge_Multi_Temp)',
            'level(Mid,Entry_Multi_Lux)']

for element in evidenceSTR:
    f1.write(element+'\n')
f1.close()

command='/opt/alchemy/infer'
params=[
    command,
    '-ms','1',                  # Run inference using MC-SAT and return probabilities for all query atoms
    '-numSolutions','10',       # Return nth SAT solution in SampleSaT
    '-saRatio','0',             # Ratio of sim. annealing steps mixed with WalkSAT in MC-SAT
    '-saTemperature','10',      # Temperature (/100) for sim. annealing step in SampleSat
    '-i',network,               # Comma-separated input .mln files
    '-r',resultFile,             # The probability estimates are written to this file.
    '-e',evidenceDB,            # The probability estimates are written to this file.
    '-q',query]        # Query atoms (comma-separated with no space)  ,e.g., cancer,smokes(x),friends(Stan,x). Query atoms are always open world.

FNULL = open(os.devnull, 'w')
out=call(params, stdout=FNULL, stderr=FNULL)

f = open(resultFile, 'r')
for line in f:
    tokens=line.split()
    activity=tokens[0].split('(')[1][0:-1]
    probability = float(tokens[1])*100
    print activity,probability

os.remove(evidenceDB)
os.remove(resultFile)

