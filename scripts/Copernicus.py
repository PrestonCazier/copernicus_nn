#Copernicus Project
#Team 3: Dawit Amare, Christopher Carlson, Shashwat Vinchurkar
#Description: This program uses Festival Speech synthesis program,
#AIML language and Pololu Servo driver program Maestro to give Copernicus
#a mind andan arm. It can easily be expanded into more bodyparts as 
#pololudriver has many output ports. AIML document can be modified to 
#include more responses. This file is a good way to see how things work
#But, it is recommended to apply this knowledge into devloping thi more.
#Better yet, ROS platform can be setup and applied with this as 
#a learning introduction
#
#CREDIT: Thanks to MATHIAS SUNARDI, MELIH ERDOGAN, JOSH SACKOS 
#and DR. PERKOWSKI
#Code snippets and AIML file developed for Jeeves robot is used here. 
#Josh Sackos created these files.They areslightly modified here.
#Mathias Sunardi helped in setting this file with Dawit Amare of Team 3
#Pololu driver control andmotions were devloped byShashwat and Christopher
import sys
import subprocess
import os
import aiml
import maestro
import fnmatch
import time
# Get current working directory
cwd = os.getcwd()

# Instantiate aiml kernel
j = aiml.Kernel()
# Load the aiml database
j.learn(cwd + '/jeeves.aiml')
j.respond('load aiml b') #finished initializingi
# Just testing
#aiml_response = j.respond('HI JEEVES')

# Print stuff out
#print aiml_response
print "Name of script: ", sys.argv[0]
print "Arguments: ", str(sys.argv)

# Reading the argument 
# argv[0] = the name of the file
# argv[1] = the first argument ...
arg = sys.argv[1]

# Variable for text to be spoken by festival
say = None

speech_txt = arg
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response

if arg  == "hello":
		say = arg
		print "say: ", say
else:
    say = aiml_response
print "Gets here: "

# Call Festival
#subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])
#subprocess.call('echo '+ aiml_response +'|festival --tts', shell=True)
#os.system('echo %s | festival --tts' % say)
print arg
print "start"
#subprocess.call(['smpleBash.sh'])   #how to call bash scriptif needed
#os.system('sh smpleBash.sh')
servo = maestro.Controller()
servo.setAccel(0,4)      #set servo 0 acceleration to 4
servo.setTarget(0,6000)  #set servo to move to center position



#The following is hard coded to show synchornization of arm servos ans speech
speech_txt = 'HI'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
servo.runScriptSub(11)
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

speech_txt = 'NAME'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
servo.runScriptSub(16)
time.sleep(1.5)
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])
time.sleep(1)

say = 'Math is Fun! Let me show you how to count.' 
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

speech_txt = '1'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
servo.runScriptSub(0)
time.sleep(2)			#delay added to sync as counting action sequence is continous.
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

speech_txt = 'COUNT2'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
#servo.runScriptSub(0)		# no need to call again as counting action is continuous.
time.sleep(0)
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

speech_txt = 'COUNT3'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
#servo.runScriptSub(0)
time.sleep(0)
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

speech_txt = 'COUNT4'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
#servo.runScriptSub(0)
time.sleep(0)
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

speech_txt = 'COUNT5'
print "arg: ", arg
# Ask AIML
aiml_response = j.respond(speech_txt)
print "aiml_response: ", aiml_response
say = aiml_response 
#servo.runScriptSub(0)
time.sleep(0)
subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])
servo.close

#END of hard coded Counting demo
#Interact with AI using Jeeves code slightly modified

while True:
	print "Ask me a question"
	question = raw_input()
	aiml_response = j.respond(question)
	subprocess.call(["festival", "--batch", "(SayText \"" + aiml_response + "\")"])


print "end"
