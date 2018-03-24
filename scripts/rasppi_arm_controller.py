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


servo = maestro.Controller()
servo.setAccel(0,4)      #set servo 0 acceleration to 4
servo.setTarget(0,6000)  #set servo to move to center position
