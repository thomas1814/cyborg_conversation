#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import time
import roslib 
import rospy
import actionlib
from std_msgs.msg import String
from cyborg_controller.msg import StateMachineAction, StateMachineGoal, StateMachineResult, StateMachineFeedback, EmotionalState, EmotionalFeedback, SystemState

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.3"
__all__ = []

class ConversationServer():
	"""ConversationServer"""
	
	current_emotion = "neutral"

	aborted = False # Set true if state needs to abort, see state loop.
	timeout = 120 # (secunds)

	# The initzialzation. Sets up all the ROS topics.
	def __init__(self):
		self.server_conversation = actionlib.SimpleActionServer(rospy.get_name() + "/conversation", StateMachineAction, execute_cb=self.server_conversation_callback, auto_start = False)
		self.server_conversation.start()
		rospy.loginfo("ConversationServer: Activated")

		self.server_follower = actionlib.SimpleActionServer(rospy.get_name() + "/follower", StateMachineAction, execute_cb=self.server_follower_callback, auto_start = False)
		self.server_follower.start()
		rospy.loginfo("FollowerServer: Activated")

		self.server_joke = actionlib.SimpleActionServer(rospy.get_name() + "/joke", StateMachineAction, execute_cb=self.server_joke_callback, auto_start = False)
		self.server_joke.start()
		rospy.loginfo("JokeServer: Activated")

		self.server_selfie = actionlib.SimpleActionServer(rospy.get_name() + "/selfie", StateMachineAction, execute_cb=self.server_selfie_callback, auto_start = False)
		self.server_selfie.start()
		rospy.loginfo("SelfieServer: Activated")

		self.server_simon_says = actionlib.SimpleActionServer(rospy.get_name() + "/simon_says", StateMachineAction, execute_cb=self.server_simon_says_callback, auto_start = False)
		self.server_simon_says.start()
		rospy.loginfo("SimonSaysServer: Activated")

		self.server_weather = actionlib.SimpleActionServer(rospy.get_name() + "/weather", StateMachineAction, execute_cb=self.server_weather_callback, auto_start = False)
		self.server_weather.start()
		rospy.loginfo("WeatherServer: Activated")

		self.emotion_publisher = rospy.Publisher("/cyborg_controller/emotional_feedback", EmotionalFeedback, queue_size=100)
		self.event_publisher = rospy.Publisher("/cyborg_controller/register_event", String, queue_size=100)
		self.speech_publisher = rospy.Publisher("/cyborg_text_to_speech/text_to_speech", String, queue_size=100)
		self.emotion_subscriber = rospy.Subscriber("/cyborg_controller/emotional_state", EmotionalState, self.emotion_callback, queue_size=100)
		self.text_subscriber = rospy.Subscriber("/text_from_speech", String, self.text_callback, queue_size=100)
		


	# Send the change (delta) in the emotional values to the controller (motion system)
	def send_emotion(self, pleasure, arousal, dominance):
		msg = EmotionalFeedback()
		msg.delta_pleasure = pleasure
		msg.delta_arousal = arousal
		msg.delta_dominance = dominance 
		self.emotion_publisher.publish(msg)


	# Called when the controller (state machine) sets the conversation state as active
	def server_conversation_callback(self, goal):
		rospy.loginfo("ConversationServer: Executing conversation state...")

		# This is the state loop
		rate = rospy.Rate(.5) # (hz)
		start = time.time() # Prevent eternal looping
		while not rospy.is_shutdown():

			# Do some conversational stuff here...

			# Checkin with the controller
			if self.server_conversation.is_preempt_requested():
				self.server_conversation.set_preempted()
				return
			if self.aborted: # This is currently always False (should be set True if the conversation state needs to abort)
				self.aborted = False
				self.server_conversation.set_aborted()
				return
			end = time.time()
			if (end - start > self.timeout):
				self.server_conversation.set_aborted()
				return
			rate.sleep()


	# Updates the current emotion when the emotion subscriber recives data from the controller (emotion system)
	def emotion_callback(self, data):
		self.current_emotion = data.to_emotional_state


	# Called when the speech to text publishes new text
	def text_callback(self, data):
		self.text = data.data
		rospy.logdebug("ConversationServer: Recived text from stt: " + str(self.text))
		self.event_publisher.publish("conversation_interest")



	# Called when the controller (state machine) sets the follower state as active
	def server_follower_callback(self, goal):
		rospy.loginfo("FollowerServer: Executing follower state...")
		self.speech_publisher.publish("Perhaps I can follow you?")
		self.server_follower.set_aborted()


		# Called when the controller (state machine) sets the joke state as active
	def server_joke_callback(self, goal):
		rospy.loginfo("JokeServer: Executing joke state...")
		self.speech_publisher.publish("Do you know any funny jokes?")
		self.server_joke.set_aborted()


	# Called when the controller (state machine) sets the selfie state as active
	def server_selfie_callback(self, goal):
		rospy.loginfo("SelfieServer: Executing selfie state...")
		self.speech_publisher.publish("Lets take a selfie!")
		self.server_selfie.set_aborted()


	# Called when the controller (state machine) sets the simon_says state as active
	def server_simon_says_callback(self, goal):
		rospy.loginfo("SimonSaysServer: Executing simon_says state...")
		self.speech_publisher.publish("Do you want to play simon says?")
		self.server_simon_says.set_aborted()


	# Called when the controller (state machine) sets the weather state as active
	def server_weather_callback(self, goal):
		rospy.loginfo("WeatherServer: Executing weather state...")
		self.speech_publisher.publish("What do you think about the weather?")
		self.server_weather.set_aborted()

