# README
This repository contains the source code for the conversation state ROS node in the NTNU Cyborg.

Node name: cyborg_conversation   
Language: Python  
Numbers of actionlib server(s): 6  

## Requirements:  
* ROS   
  
  
## Features:  
* The conversation state: The Cyborg`s conversation state. Available at actionlib server topic cyborg_conversation/conversation. It stays here.   
* The follower state: The Cyborg says: "Perhaps I can follow you?". Available at actionlib server topic cyborg_conversation/follower. 
* The joke state: The Cyborg says: "Do you know any funny jokes?". Available at actionlib server topic cyborg_conversation/joke. 
* The selfie state: The Cyborg says: "Lets take a selfie!". Available at actionlib server topic cyborg_conversation/selfie.  
* The simon_says state: The Cyborg says: "Do you want to play simon says?". Available at actionlib server topic cyborg_conversation/simon_says.  
* The weather state: The Cyborg says: "What do you think about the weather?". Available at actionlib server topic cyborg_conversation/weather.   


## Usage:
$ rosrun cyborg_conversation conversation.py
