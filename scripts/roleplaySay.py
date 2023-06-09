#!/usr/bin/env python3

import rospy
import gtts

from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
from roleplay_storytelling.srv import RoleplaySay

speech_mode = "QT"

def handle_roleplay_say(req):
    print(f'Running handle_roleplay_say\n')
    #Something
    global speech_mode
    if speech_mode=="QT":
        try:
            speechVolume = rospy.ServiceProxy('/qt_robot/setting/setVolume', setting_setVolume)
            resp = speechVolume(60)
            print(f"Volume response: {resp}\n")
            speechConfig = rospy.ServiceProxy('/qt_robot/speech/config',speech_config)
            lang = req.language
            resp = speechConfig(lang,0,0)
            print(f'config result: {resp}\n')
            speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
            resp = speechSay(req.message)
        except:
            print(f'ERROR when calling TTS services\n')
        #return
    else:
        if speech_mode=="QT+GTTS":
            tts = gtts.gTTS(text, lang=language.googletrans)
            tts.save("./hola.mp3")
            speechSay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
            audioFile = ['halo','']
            resp = speechSay('halo','')
            print(f"Audio request response: {resp}\n")
            #return
        #else:
            #return
    return "ok"

def roleplay_say_server():
    rospy.init_node('roleplay_say_server')
    s = rospy.Service('roleplay_say', RoleplaySay, handle_roleplay_say)
    print("Ready to say something.")
    rospy.spin()

if __name__ == "__main__":
    roleplay_say_server()
