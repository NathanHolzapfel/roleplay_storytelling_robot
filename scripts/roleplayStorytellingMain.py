#!/usr/bin/env python3

import logging
import rospy
import smach
import requests
import json
import random
import math
from datetime import datetime

#May have to install these:
from helpers import *
from googletrans import Translator
import gtts
from playsound import playsound
import speech_recognition as sr
import deepl
from std_msgs.msg import String
from roleplay_storytelling.srv import RoleplaySay
#from qt_robot_interface.srv import *
#from qt_vosk_app.srv import *
# # #

#laptop IP adress:
# 10.0.2.15
# 127.0.0.1

# "QT": QT voice, "QT+GTTS": QT plays gtts audio, "None": No speech
#speech_mode = "QT"
speech_mode =  "QT"

total_n_translated = 0
total_n = 0
new_n_translated = 0
new_n = 0
total_n_displayed = 0
difficulty_level = 0.5 #should be between 0.1 and 1

class language:
    def __init__(self,name,deepl,qt,googletrans):
        self.name = name
        self.deepl = deepl
        self.qt = qt
        self.googletrans = googletrans


class languages:
    ENGLISH = language("english","en-gb","en-US","en")
    FRENCH = language("french","fr","fr-FR","fr")
    GERMAN = language("german","de","de-DE","de")

#Console text colors
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def generate_text(scenario):
    data_set = {"prompt": scenario}
    json_dump = json. dumps(data_set)
    json_object = json. loads(json_dump)
    r = requests.post('http://127.0.0.1:5000/api/v1/generate', json = json_object)
    #r = requests.post('https://ninety-otters-thank-104-154-165-162.loca.lt/api/v1/generate', json = json_object)
    results = r.json()['results']
    return results[0]['text']

def say(text, language):
    #print(f'say: function start\n')
    rospy.wait_for_service('roleplay_say')
    try:
        roleplay_say = rospy.ServiceProxy('roleplay_say', RoleplaySay)
        roleplay_say(text, language.qt)
        #return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    #print(f'say: function end\n')

def L_translate(text, source_lang, target_lang):
    if source_lang==target_lang:
        return text
    if text == "":
        return "EMPTY_TRANSLATION"

    api_deepL = True #False to use Google Translate, True for DeepL
    if api_deepL:
        auth_key = "xxxx_xxxx..." #INSERT YOUR OWN KEY
        translator = deepl.Translator(auth_key)
        tgt_lg = target_lang.deepl
        src_lg = source_lang.deepl
        if src_lg == "en-gb":
            src_lg = "en"
        new_text = translator.translate_text(text, source_lang = src_lg, target_lang=tgt_lg).text
    else:
        translator = Translator()
        tgt_lg = target_lang.googletrans
        new_text = translator.translate(text, dest=target_lang).text
    return new_text

def translate_and_print(text, language):
    if language.name == "english":
        print(text)
    else:
        translation = L_translate(text,languages.ENGLISH,language)
        print(translation)

def RemoveBadCharacters(text):
    characterlist = " abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789,;.:?!-ÀÈÉÇéèàÂÊÎÔÛâêîôûç()%'@+-*/()[]$£" + '"'
    punctuationlist = ".?!"
    clean_text = ""
    #Remove unrecognized characters
    for character in text:
        if characterlist.find(character) != -1:
            clean_text = clean_text + character
    #Automatic punctuation
    if clean_text == "":
        return clean_text
    if punctuationlist.find(clean_text[len(clean_text)-1]) == -1:
        clean_text = clean_text + "."
    return clean_text

def get_context(id):
    context = ""
    if id=="1":
        #Starless Sea
        context = "The cyclopean ruins stand high above your village. The ruins are teeming with vile creatures and horrors. Your village has been under attack from an unknown evil that lurks within the keep. You have rallied dozens of your fellow villagers into a party to explore the keep in hopes of stopping the evil inside. Your party is comprised of simple commoners, including men, women, and children. You are your party."
    elif id=="2":
        #Adrenaline Rush
        context = "After the Fall, a dangerous sport became popular amongst the desperate youth of the apocalypse. Called 'jetboarding', the sport comprises of racing modified hoverboards around the crumbling structures of rusted and rotting rollercoasters. The death rate is incredibly high, but without hope for a future, many prefer to die in pursuit of reckless glory. You are Pierre, a young jetboarder new to the game. These days, with infrastructure crumbling, jetboarding and other dangerous sports were becoming the only ways kids from the wrong side of the tracks can obtain enough money to provide for themselves."
    elif id=="3":
        #Skeleton
        context = "You are Larry. You are an undead skeleton. You remember your name. You forget everything else about your former life. You are dressed in rusted chainmail. You carry a curved sword. Edgar is a necromancer. Edgar raised you from the dead as a skeleton. You are compelled to obey Edgar’s commands. Edgar is an old man. Edgar is dressed in a black robe, and wields an ornate magical staff."
    else:
        #Insurrection
        context = "You are Amanda. You are a covert agent of the government of Larion. You are highly trained in combat, propaganda, covert surveillance, and all the skills of the spy trade. You are in the Kyton Republic to start a revolution against its corrupt government. You have plenty of money and specialized equipment, including weapons and surveillance gear. Your cover identity is that you are a documentary filmmaker, shooting a streaming series on Kyton history. Your most promising contact within the Kyton resistance is a mysterious figure whose alias is The Craxil."
    return context

def get_prompt(id):
    prompt = ""
    if id=="1":
        #Starless Sea
        prompt = "Since time immemorial, you and your people have toiled in the shadow of the cyclopean ruins. Of mysterious origins and the source of many a superstition, they have always been considered a secret best left unknown by the folk of your hamlet. But now something stirs beneath the crumbling blocks. Beastmen howl in the night and your fellow villagers are snatched from their beds. The time for retribution has come! With no militia to provide protection, you have rallied a dozen or so of your fellow commoners—simple tradesmen, craftsmen, and farmers who are brave to take a stand against the evil assailing your village. All of you are unskilled but there is safety and power in numbers, and hopefully, this party can help one another survive. You and your companions stand before the ruined keep, your meager weapons at the ready. The ruins squat atop a low, craggy hill, its walls of toppled stone and massive granite blocks hinting at forgotten battles and the clash of mighty armies. Now the ruins seem host only to creeping vines and the foul miasma that drifts down from the keep. The air is overrun with pestilence. Fat flies bite at you incessantly, and clouds of small black insects choke your every breath. The long-abandoned land is choked with thorny vines that drape the sickly trees and hang from the ruined walls. There is an odor of rot and decay as if the hill itself were decomposing from within. The keep’s massive wall has collapsed here in the northeast part of the wall, spilling cyclopean stone blocks down the rocky slope. The blocks are precariously balanced atop one another, like a titan’s game of dice. Upon searching the rubble you discover a narrow, rocky shaft descending down before opening into a small chamber. Your party squeezes down through the narrow opening, going single file, dropping into the chamber below. The air is choked with chalky dust and the pervasive smell of rot. A single shaft of light cuts through the swirling dust to illuminate an enormous stone door set in the wall. The door is circumscribed in runes. At the center of the door is a large pentagram inscribed within a circle. Both the runes and the pentagram are set with silver and seem to glimmer faintly in the dim light."
    elif id=="2":
        #Adrenaline Rush
        prompt = "The wind was strong up here, so strong it overwhelmed your senses. It howled through your ears, chilled your skin, and whipped you with almost enough force to blow you off your jetboard. Thankfully your feet were firmly attached with maglock boots that kept you from falling into the void. You squinted at the dilapidated tracks ahead, a hulking remnant of what once was a sturdy rollercoaster. Now it was a rotting carcass, barely capable of supporting the carts that once raced around its length. It was, however, capable of supporting you. This is what every kid from the shantytown of The Bricks dreamed of: their chance to get a foothold into the death-defying world of professional jetboarding. The coaster you balanced on dominated our skyline and our dreams. Back in the day, it was called the Gold Striker. Now, due to its rusting steel strips and crumbling wooden tracks, along with its terrifying plunge into an entirely darkened mineshaft, it bore the moniker of the Coffin Coaster. You shot a glance to your side, where your opponent, Riley Mulligan, was leaning into position. To the side, standing on the platform that held the now-useless coaster carts, were the judges. Behind them were the scouts: the ones who could pluck us out of our lives of hunger into the world of riches and glory. Insane to think that this was it; insane to think that years of training on ancient jetboards and makeshift rails were about to pay off. Riley was silent for once, her eyes closed and face pale. You could see a slight tremble in her shoulders and legs, but the grin twitching her lips told you that it was not nerves, but excitement. She held her slim body near the tip of her board, suicidally close to the edge for maximum speed. Ever since we were kids, she had a penchant for incredible recklessness. She was always careening headlong towards death, never looking back, never thinking twice about what might happen next. Her blue eyes were always turned towards some distant future, and the hunger in them made you nervous. And this was her chance, just as it was mine. One of us would take the win, and one of us would go home empty-handed. We waited there, an instant of eternity, till we heard the blow of the whistle that signaled the beginning. Your heart kicked into high gear, adrenaline pumping through your veins. A split-second calculation flashed through you: you could try to beat Riley on pure speed, or impress the judges with acrobatics and trickery. You built your jetboard with scraps as an all-rounder, so theoretically either approach would work. But you knew that Riley had practiced more than anyone else in The Bricks, and her insane recklessness made her unpredictable. There was another option too: jetboarding had very few rules. Though direct harm to one's opponent is prohibited, a whole range of things are allowed, ranging from dirty tactics to sabotaging the track. All these flew through your body more as impulse than thought, and your body made its decision. You tipped forward on your board and"
    elif id=="3":
        #Skeleton
        prompt = '“Rise! Rise, and serve!”' + " The voice is the first thing you hear, awakening you from a sleep of centuries. It’s raspy with age, but sure and commanding. You have no eyes to open, but somehow you see its owner clearly, a stooped old man in a black robe. Bone rattles against metal as you rise from the stone slab where you’ve lain for…you don’t know how long. You remember that your name was Larry, in life, but nothing else. It seems you were a warrior, though. You’re still clad in the rusted remains of chainmail, and your bony fingers clutch an ancient curved sword. You rise, and stand at attention before your new master. "
        prompt = prompt + '“Good, good.” He says, not unkindly. “You' + "’re still mostly intact. Come, follow me. We have much work to do."
        prompt = prompt + '” You feel a compulsion to obey his command. You take a creaking step forward, then another. Your muscle and sinew rotted away long ago, but some dark power holds you together. You walk slowly after the robed man.'
        prompt = prompt + " It’s pitch black in this crypt, but somehow you see clearly as you make your way down a narrow stone corridor. "
        prompt = prompt + '“My name is Edgar.” He says. “Now,' + " let’s rouse your companions." + '” Looking around, you see that'
        prompt = prompt + " you’ve shared this tomb with dozens of other skeletons, lying on stone slabs, all armed and armored like you. Edgar stops by the nearest skeleton. His hand glows deep purple as he makes a gesture in the air."
        prompt = prompt + ' “Rise! Rise, and serve!” Another skeleton stirs.'
    else:
        #Insurrection
        prompt = "The Kyton Republic exists somewhere in the murky grey area, between a functional representative democracy and an oligarchy dominated by a few well-connected individuals. The President is entering his seventh term this year, and it’s generally assumed he’ll stay in office until he dies. The country might be fairly called a banana republic, if bananas had any chance of growing in its cold, mountainous climate. Foreign business is welcome, provided the right bribes are paid and the company doesn’t agitate too much for the rights of those it employs. The Army is a force unto itself, well equipped and trained, although corruption in the ranks is rampant. But overall It’s not a terrible place to live, unless you cross someone powerful. Life in Kyton goes on, in its messy way, and most people just go along, and get along. But your job is to bring it all crashing down. You’re Amanda. Officially, you’re in Kyton as a documentary filmmaker, shooting an in-depth streaming series on Kyton history. But secretly, you’re a covert agent of the government of Larion. The powers that sign your paycheck have decided, in their unquestioned wisdom, that Kyton needs a change in regime. And that’s exactly the sort of task you’ve trained all your life to accomplish. You have substantial funding, both in untraceable foreign bank accounts as well as several fat duffel bags of hard cash in the local currency. You have a number of interesting toys, too, from weapons, to surveillance gear, to a document printing apparatus capable of creating perfect forged credentials. And you have a number of potential contacts within Kyton’s dissident and revolutionary underground, who oppose the regime for reasons as varied as they are impassioned. Right now you’re with your camera crew, shooting exterior footage of one of Kyton’s many historic cathedrals. But you’re also waiting to hear from a shadowy figure you know only as "
        prompt = prompt + '“The Craxil”.' + " You look around. There’s an elderly woman feeding pigeons from a park bench. A uniformed policeman stands on a street corner, facing away from you. A skinny, pockmarked teenager sells hot food from a pushcart. A young couple hold hands, admiring the cathedral and each other. An attractive woman in sunglasses and a blue sweater rides by on a bicycle. Any of them could be The Craxil. You keep moving, shooting video of the cathedral’s intricately carved facade, while you wait for something to happen."
        prompt = prompt + '“Hey!”'
    return prompt

# define state Initialization
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],output_keys=['scenario_out','context_out','difficulty_out','usr_lang_out','trg_lang_out'])
        self.counter = 0

    def execute(self, userdata):
        #rospy.loginfo('Executing state Initialization')

        print(f"{bcolors.OKCYAN}Please choose your mastered language(1:{bcolors.OKGREEN}English{bcolors.OKCYAN}, 2:{bcolors.OKGREEN}French{bcolors.OKCYAN}, 3:{bcolors.OKGREEN}German{bcolors.OKCYAN}):{bcolors.ENDC}")
        lang_id = input()
        print(f"{bcolors.OKCYAN}Please choose your learning language(1:{bcolors.OKGREEN}English{bcolors.OKCYAN}, 2:{bcolors.OKGREEN}French{bcolors.OKCYAN}, 3:{bcolors.OKGREEN}German{bcolors.OKCYAN}):{bcolors.ENDC}")
        lang_id2 = input()
        if lang_id=="1":
            usr_lang = languages.ENGLISH
        elif lang_id=="2":
            usr_lang = languages.FRENCH
        elif lang_id=="3":
            usr_lang = languages.GERMAN
        else:
            print(f"Error, defaulting to english")
            usr_lang = languages.ENGLISH
        if lang_id2=="1":
            trg_lang = languages.ENGLISH
        elif lang_id2=="2":
            trg_lang = languages.FRENCH
        elif lang_id2=="3":
            trg_lang = languages.GERMAN
        else:
            print(f"Error, defaulting to english")
            trg_lang = languages.ENGLISH
        
        userdata.usr_lang_out = usr_lang
        userdata.trg_lang_out = trg_lang

        global difficulty_level
        #Setting difficulty
        print(f"{bcolors.OKCYAN}Please choose starting difficulty (1:{bcolors.OKGREEN}Normal Mode{bcolors.OKCYAN}, 2:{bcolors.WARNING}Advanced Mode{bcolors.OKCYAN}):{bcolors.ENDC}\n")
        nb_input = input()
        if nb_input == "1":
            print(f'{bcolors.OKCYAN}You have chosen {bcolors.OKGREEN}Normal difficulty{bcolors.ENDC}\n')
            difficulty_level = 0.2
            userdata.difficulty_out = 1
        else:
            if nb_input == "2":
                print(f'{bcolors.OKCYAN}You have chosen {bcolors.WARNING}Advanced difficulty{bcolors.ENDC}\n')
                difficulty_level = 1.0
                userdata.difficulty_out = 2
            else:
                print(f"{bcolors.OKCYAN}Failure to understand user's intention, defaulting to Normal difficulty{bcolors.ENDC}\n")
                difficulty_level = 0.2
                userdata.difficulty_out = 1

        #Requesting context
        context = input(f"{bcolors.OKCYAN}Please enter context (1: Starless Sea, 2: Adrenaline Rush, 3: Skeleton, 4: Insurrection):{bcolors.ENDC}\n")
        #print(f'{bcolors.OKCYAN}You typed {bcolors.ENDC}{bcolors.BOLD}{context}{bcolors.ENDC}')
        #userdata.context_out = context + "\n"
        userdata.context_out = get_context(context) + "\n"
        #Requesting initial scenario
        #initial_scenario = input(f"{bcolors.OKCYAN}Please enter initial scenario:{bcolors.ENDC}\n")
        #print(f'{bcolors.OKCYAN}You typed {bcolors.ENDC}{bcolors.BOLD}{initial_scenario}{bcolors.ENDC}')
        #userdata.scenario_out = initial_scenario + "\n"
        userdata.scenario_out = get_prompt(context) + "\n"
        print(f"{L_translate(get_prompt(context), languages.ENGLISH, usr_lang)}")
        ready = "When you are ready, press"
        print(f"{bcolors.OKCYAN}{L_translate(ready,languages.ENGLISH,usr_lang)} {bcolors.OKGREEN}[Enter].{bcolors.ENDC}")
        r = input()
        return 'outcome1'

# define state StoryGeneration
class StoryGeneration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['scenario_in','context_in'],output_keys=['scenario_out','new_scenario','help_level_out'])
        self.memory1 = ""
        self.memory2 = ""
        self.memory3 = ""
        self.memory4 = ""
        self.memory5 = ""
        self.memory6 = ""
        self.memory7 = ""
        self.memory8 = ""
        self.memory9 = ""
        self.memory10 = ""
        self.memory_counter = 0

    def execute(self, userdata):
        #rospy.loginfo('Executing state StoryGeneration')

        #aitext = "This will be AI-Generated text.\n"
        #TODO Limit story input size
        

        maxattempts = 10
        attempts = 0
        seed_scenario = userdata.context_in
        
        temp_counter = self.memory_counter
        for i in range(10):
            temp_counter += 1
            #print(f"[DEBUG]Adding memory {temp_counter} to input story")
            if temp_counter == 1:
                seed_scenario = seed_scenario + self.memory1
            if temp_counter == 2:
                seed_scenario = seed_scenario + self.memory2
            if temp_counter == 3:
                seed_scenario = seed_scenario + self.memory3
            if temp_counter == 4:
                seed_scenario = seed_scenario + self.memory4
            if temp_counter == 5:
                seed_scenario = seed_scenario + self.memory5
            if temp_counter == 6:
                seed_scenario = seed_scenario + self.memory6
            if temp_counter == 7:
                seed_scenario = seed_scenario + self.memory7
            if temp_counter == 8:
                seed_scenario = seed_scenario + self.memory8
            if temp_counter == 9:
                seed_scenario = seed_scenario + self.memory9
            if temp_counter == 10:
                seed_scenario = seed_scenario + self.memory10
                temp_counter = 0 #cycling back
        seed_scenario = seed_scenario + userdata.scenario_in

        while True:
            aitext = generate_text(seed_scenario)
            attempts += 1
            if len(aitext)>3 or attempts >= maxattempts:
                break
        if len(aitext)<=3:
            print(f"{bcolors.FAIL}Failure to generate story. Ending session.{bcolors.ENDC}\n")
            raise InterruptedError("Story generation failed.")
        
        #Taking only complete sentences
        sentence_text = ""
        remaining_text = aitext
        #Removing THE END if need be
        end_idx = remaining_text.find("THE END")
        if end_idx != -1:
            remaining_text = remaining_text[:end_idx-1] + remaining_text[:end_idx+6]
        for iteration in range(50):
            dot_idx = remaining_text.find(".")
            qu_idx = remaining_text.find("?")
            ex_idx = remaining_text.find("!")
            idx_list = [dot_idx, qu_idx, ex_idx]
            for idx_idx, idx in enumerate(idx_list):
                if idx == -1 or idx < 4: #-1: not found, <4: sentence too short
                    idx_list[idx_idx] = 1000000
            min_idx = min(idx_list) + 1
            if min_idx >= 1000000:
                break
            sentence_text = sentence_text + remaining_text[:min_idx]
            remaining_text = remaining_text[min_idx:]
        #Test:
        #sentences.append(remaining_text)
        aitext = sentence_text + "\n"

        userdata.new_scenario = aitext
        userdata.help_level_out = 0

        #saving story for next generation requests
        self.memory_counter += 1
        if self.memory_counter == 1:
            self.memory1 = userdata.scenario_in + aitext
        if self.memory_counter == 2:
            self.memory2 = userdata.scenario_in + aitext
        if self.memory_counter == 3:
            self.memory3 = userdata.scenario_in + aitext
        if self.memory_counter == 4:
            self.memory4 = userdata.scenario_in + aitext
        if self.memory_counter == 5:
            self.memory5 = userdata.scenario_in + aitext
        if self.memory_counter == 6:
            self.memory6 = userdata.scenario_in + aitext
        if self.memory_counter == 7:
            self.memory7 = userdata.scenario_in + aitext
        if self.memory_counter == 8:
            self.memory8 = userdata.scenario_in + aitext
        if self.memory_counter == 9:
            self.memory9 = userdata.scenario_in + aitext
        if self.memory_counter == 10:
            self.memory10 = userdata.scenario_in + aitext
            self.memory_counter = 0 #cycling back
        
        userdata.scenario_out = userdata.scenario_in + aitext

        return 'outcome1'
        
# define state StoryTTS
class StoryTTS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['new_story_in','user_lang_in','trgt_lang_in','difficulty_in','nb_translated_in'],output_keys=['nb_translated_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state StoryTTS')

        translator = Translator()

        #Separating into sentences.
        sentences = list()
        remaining_text = userdata.new_story_in
        for iteration in range(50):
            dot_idx = remaining_text.find(".")
            qu_idx = remaining_text.find("?")
            ex_idx = remaining_text.find("!")
            idx_list = [dot_idx, qu_idx, ex_idx]
            for idx_idx, idx in enumerate(idx_list):
                if idx == -1 or idx < 4: #-1: not found, <4: sentence too short
                    idx_list[idx_idx] = 1000000
            min_idx = min(idx_list) + 1
            if min_idx >= 1000000:
                break
            sentences.append(remaining_text[:min_idx])
            remaining_text = remaining_text[min_idx:]
        #Test:
        sentences.append(remaining_text)

        story_continues = "The story continues, please listen."
        new_story_continues = L_translate(story_continues, languages.ENGLISH, userdata.user_lang_in)
        print(f"{bcolors.OKCYAN}{new_story_continues}{bcolors.ENDC}\n")
        say(new_story_continues, userdata.user_lang_in)

        #Translation and output
        # shortest_idx = 0
        # shortest_length = 9999999
        # length_threshold = 15
        # for idx_sentence, sentence in enumerate(sentences):
        #     if len(sentence)<=shortest_length and len(sentence)>length_threshold:
        #         shortest_length = len(sentence)
        #         shortest_idx = idx_sentence
        
        # nb_translated = userdata.nb_translated_in
        # global new_n, new_n_translated, total_n, total_n_translated
        # new_n = 0
        # new_n_translated = 0
        # for idx_sentence, sentence in enumerate(sentences):
        #     if idx_sentence == shortest_idx or userdata.difficulty_in==2:
        #         if userdata.difficulty_in!=2:
        #             new_sentence = L_translate("The next sentence is translated.", "en", userdata.user_lang_in)
        #             say(new_sentence, userdata.user_lang_in)
        #         new_sentence = L_translate(sentence, languages.ENGLISH, userdata.trgt_lang_in)
                
        #         say(new_sentence, userdata.trgt_lang_in)
        #     else:
        #         new_sentence = L_translate(sentence, languages.ENGLISH, userdata.user_lang_in)
                
        #         say(new_sentence, userdata.user_lang_in)
        #         nb_translated[0] = nb_translated[0] + len(sentence)
        #         new_n_translated = new_n_translated + len(sentence)
        #     nb_translated[1] = nb_translated[1] + len(sentence)
        #     new_n = new_n + len(sentence)
        ############

        #Computing optimal translated sent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ence combination
        len_sentences = [len(s) for s in sentences]
        goal_len = difficulty_level* sum(len_sentences)
        max_seed = pow(2,len(sentences))-1
        current_seed = 0
        minimal_diff = 0
        while(True):
            current_seed += 1
            current_len = 0
            for idx_L, L in enumerate(len_sentences):
                local_seed = pow(2,idx_L)
                if local_seed&current_seed == local_seed:
                    current_len += L
            if abs(current_len-goal_len)<minimal_diff or current_seed==1:
                optimal_seed = current_seed
                minimal_diff = abs(current_len-goal_len)
            if current_seed > max_seed:
                break
        
        nb_translated = userdata.nb_translated_in
        global new_n, new_n_translated, total_n, total_n_translated
        new_n = 0
        new_n_translated = 0
        for idx_sentence, sentence in enumerate(sentences):
            local_seed = pow(2,idx_sentence)
            if local_seed & optimal_seed == local_seed:
                #TODO: Gesture?
                # if userdata.difficulty_in!=2:
                #     new_sentence = L_translate("The next sentence is translated.", "en", userdata.user_lang_in)
                #     say(new_sentence, userdata.user_lang_in)
                new_sentence = L_translate(sentence, languages.ENGLISH, userdata.trgt_lang_in)
                
                say(new_sentence, userdata.trgt_lang_in)
            else:
                new_sentence = L_translate(sentence, languages.ENGLISH, userdata.user_lang_in)
                
                say(new_sentence, userdata.user_lang_in)
                nb_translated[0] = nb_translated[0] + len(sentence)
                new_n_translated = new_n_translated + len(sentence)
            nb_translated[1] = nb_translated[1] + len(sentence)
            new_n = new_n + len(sentence)

        userdata.nb_translated_out = nb_translated
        return 'outcome1'

# define state StoryRecognition
class StoryRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4','outcome5'],input_keys=['scenario_in','help_level_in','trgt_lang_in','usr_lang_in'],output_keys=['help_level_out','new_scenario'])
        self.counter = 0

    def execute(self, userdata):
        #rospy.loginfo('Executing state StoryRecognition')
        speech_recognition_enabled = False
        global new_n, new_n_translated, total_n, total_n_translated
        raw1 = "Note: When writing a story scene, please write sentences in"
        tl1= L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
        raw2 = f"{userdata.trgt_lang_in.name}"
        tl2 = L_translate(raw2, languages.ENGLISH, userdata.usr_lang_in)
        raw3 = "with punctuation and second person use ('you')"
        tl3 = L_translate(raw3, languages.ENGLISH, userdata.usr_lang_in)
        print(f"\n{bcolors.OKCYAN}{tl1} {bcolors.WARNING}{tl2}{bcolors.OKCYAN},{bcolors.BOLD} {tl3}{bcolors.ENDC}")
        help_use = 'transcript (text)'
        if userdata.help_level_in>0:
            help_use = 'translation'
        #help_use_tl = L_translate(help_use, languages.ENGLISH, userdata.usr_lang_in)
        raw1 = "Please enter your next action"
        tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
        raw2 = "for " + help_use
        tl2 = L_translate(raw2, languages.ENGLISH, userdata.usr_lang_in)
        raw3 = "to end session"
        tl3 = L_translate(raw3, languages.ENGLISH, userdata.usr_lang_in)
        raw4 = "to describe story action verbally"
        tl4 = L_translate(raw4, languages.ENGLISH, userdata.usr_lang_in)
        raw5 = "or something else to describe story action"
        tl5 = L_translate(raw5, languages.ENGLISH, userdata.usr_lang_in)
        raw6 = "through the keyboard directly"
        tl6 = L_translate(raw6, languages.ENGLISH, userdata.usr_lang_in)
        raw7 = "You can also adjust the difficulty, aka as the amount of text translated in"
        tl7 = L_translate(raw7, languages.ENGLISH, userdata.usr_lang_in)
        raw8 = f"{userdata.trgt_lang_in.name}, by typing"
        tl8 = L_translate(raw8, languages.ENGLISH, userdata.usr_lang_in)
        raw9 = "or"
        tl9 = L_translate(raw9, languages.ENGLISH, userdata.usr_lang_in)

        input_prompt1 = (f"{bcolors.OKCYAN}{tl1}({bcolors.OKGREEN}help{bcolors.OKCYAN} {tl2}," +
                        f" {bcolors.OKGREEN}end {bcolors.OKCYAN}{tl3}, {bcolors.OKGREEN}speak {bcolors.OKCYAN} {tl4}" +
                        f" {tl5} {tl6}):{bcolors.ENDC}\n")
        input_prompt2 = (f"{bcolors.OKCYAN}{tl1} ({bcolors.OKGREEN}help{bcolors.OKCYAN} {tl2}," +
                        f" {bcolors.OKGREEN}end {bcolors.OKCYAN}{tl3}, {tl5}):{bcolors.ENDC}\n")
        difficulty_prompt = (f"{bcolors.OKCYAN}{tl7} " +
                            f" {tl8} '{bcolors.OKGREEN}+{bcolors.OKCYAN}' {tl9} '{bcolors.OKGREEN}-{bcolors.OKCYAN}'.{bcolors.ENDC}\n")
        if speech_recognition_enabled:
            userinput = input(f"{input_prompt1}{difficulty_prompt}")
        else:
            userinput = input(f"{input_prompt2}{difficulty_prompt}")
        raw1 = "You typed"
        tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
        print(f'{bcolors.OKCYAN}{tl1} {bcolors.ENDC}{bcolors.BOLD}{userinput}{bcolors.ENDC}')
        
        if userinput == "help":
            if userdata.help_level_in == 0:
                userdata.help_level_out = 1
                return 'outcome1'
            else:
                userdata.help_level_out = 2
                return 'outcome4'
        else:
            total_n = total_n + new_n
            total_n_translated = total_n_translated + new_n_translated
            if userinput == "end":
                return 'outcome3'
            else:
                #----------------------------------Prototype--------------------------------------------
                if speech_recognition_enabled and userinput == "speak":
                    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
                    resp = recognize(userdata.trgt_lang_in.qt, ['blue', 'green', 'red'], 10)
                    userinput = resp.transcript
                    print(f"I got:{resp} with transcript being: {userinput}\n")
                    userinput = userinput.capitalize() #converts first character to upper case
                    userinput = userinput + "."
                else:
                    global difficulty_level

                    if userinput=="+":
                        if difficulty_level <=0.8:
                            difficulty_level += 0.2
                            raw1 = "Translation Difficulty is now"
                            tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
                            print(f"{tl1} {round(difficulty_level*100)}%.")
                        else:
                            difficulty_level = 1.0
                        if difficulty_level == 1.0:
                            raw2 = "Translation Difficulty has reached maximum level"
                            tl2 = L_translate(raw2, languages.ENGLISH, userdata.usr_lang_in)
                            print(f"{tl2} (100%).")
                        return 'outcome5'
                    elif userinput=="-":
                        if difficulty_level >=0.4:
                            difficulty_level -= 0.2
                            raw1 = "Translation Difficulty is now"
                            tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
                            print(f"{tl1} {round(difficulty_level*100)}%.")
                        else:
                            difficulty_level = 0.2
                        if difficulty_level == 0.2:
                            raw2 = "Translation Difficulty has reached minimum level"
                            tl2 = L_translate(raw2, languages.ENGLISH, userdata.usr_lang_in)
                            print(f"{tl2} (20%).")
                        return 'outcome5'
                userinput = RemoveBadCharacters(userinput)
                #print(f'{bcolors.OKCYAN}You entered, after correction: {bcolors.ENDC}{bcolors.BOLD}{userinput}{bcolors.ENDC}')
                userdata.new_scenario = userinput
                return 'outcome2'

# define state StoryHelp
class StoryHelp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['difficulty_in','new_story_in','trgt_lang_in','user_lang_in','nb_translated_in'],output_keys=['nb_translated_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state StoryHelp')

        translator = Translator()

        #Separating into sentences.
        sentences = list()
        remaining_text = userdata.new_story_in
        for iteration in range(50):
            dot_idx = remaining_text.find(".")
            qu_idx = remaining_text.find("?")
            ex_idx = remaining_text.find("!")
            idx_list = [dot_idx, qu_idx, ex_idx]
            for idx_idx, idx in enumerate(idx_list):
                if idx == -1 or idx < 4: #-1: not found, <4: sentence too short
                    idx_list[idx_idx] = 1000000
            min_idx = min(idx_list) + 1
            if min_idx >= 1000000:
                break
            sentences.append(remaining_text[:min_idx])
            remaining_text = remaining_text[min_idx:]

        story_continues = "The story continues:"
        print(f"{bcolors.OKCYAN}{story_continues}{bcolors.ENDC}\n")
        new_story_continues = L_translate(story_continues, languages.ENGLISH, userdata.user_lang_in)
        say(new_story_continues, userdata.user_lang_in)
        
        len_sentences = [len(s) for s in sentences]
        goal_len = difficulty_level* sum(len_sentences)
        #print(f"DEBUG: goal_len = {goal_len}")
        max_seed = pow(2,len(sentences))-1
        current_seed = 0
        minimal_diff = 0
        while(True):
            current_seed += 1
            current_len = 0
            #print(f"DEBUG: seed = {current_seed}")
            for idx_L, L in enumerate(len_sentences):
                local_seed = pow(2,idx_L)
                if local_seed&current_seed == local_seed:
                    current_len += L
            if abs(current_len-goal_len)<minimal_diff or current_seed==1:
                optimal_seed = current_seed
                minimal_diff = abs(current_len-goal_len)
            if current_seed > max_seed:
                break
        #print(f"DEBUG: optimal seed = {bin(optimal_seed)}")
        nb_translated = userdata.nb_translated_in
        global new_n, new_n_translated, total_n, total_n_translated, total_n_displayed
        new_n = 0
        new_n_translated = 0
        for idx_sentence, sentence in enumerate(sentences):
            local_seed = pow(2,idx_sentence)
            if local_seed & optimal_seed == local_seed:
                #if userdata.difficulty_in!=2:
                #    new_sentence = L_translate("The next sentence is translated.", "en", userdata.user_lang_in)
                #    say(new_sentence, userdata.user_lang_in)
                new_sentence = L_translate(sentence, languages.ENGLISH, userdata.trgt_lang_in)
                print(f"{bcolors.WARNING}{new_sentence}{bcolors.ENDC}")
                say(new_sentence, userdata.trgt_lang_in)

            else:
                #new_sentence = translator.translate(sentence, dest=userdata.user_lang_in).text
                new_sentence = L_translate(sentence, languages.ENGLISH, userdata.user_lang_in)
                print(f"{bcolors.OKGREEN}{new_sentence}{bcolors.ENDC}")
                say(new_sentence, userdata.user_lang_in)
                nb_translated[0] = nb_translated[0] + len(sentence)
                new_n_translated = new_n_translated + len(sentence)
            nb_translated[1] = nb_translated[1] + len(sentence)
            new_n = new_n + len(sentence)
            total_n_displayed += len(sentence)
        return 'outcome1'

class StoryHelp2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['new_story_in','trgt_lang_in','user_lang_in','nb_translated_in'],output_keys=['nb_translated_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state StoryHelp')

        translator = Translator()

        #Separating into sentences.
        sentences = list()
        remaining_text = userdata.new_story_in
        for iteration in range(50):
            dot_idx = remaining_text.find(".")
            qu_idx = remaining_text.find("?")
            ex_idx = remaining_text.find("!")
            idx_list = [dot_idx, qu_idx, ex_idx]
            for idx_idx, idx in enumerate(idx_list):
                if idx == -1 or idx < 4: #-1: not found, <4: sentence too short
                    idx_list[idx_idx] = 1000000
            min_idx = min(idx_list) + 1
            if min_idx >= 1000000:
                break
            sentences.append(remaining_text[:min_idx])
            remaining_text = remaining_text[min_idx:]

        #Translation and output
        repeat = "Let me repeat:"
        new_repeat = L_translate(repeat, languages.ENGLISH, userdata.user_lang_in)
        print(f"{bcolors.OKCYAN}{new_repeat}{bcolors.ENDC}\n")
        say(new_repeat, userdata.user_lang_in)
        #tts = gtts.gTTS(new_repeat, lang=userdata.user_lang_in)
        #tts.save("./hola.mp3")
        #if playsound_toggle:
        #    playsound("./hola.mp3")
        #random_number = random.randint(1,len(sentences))
        global new_n, new_n_translated, total_n, total_n_translated
        nb_translated= userdata.nb_translated_in
        new_n_translated = 0
        for idx_sentence, sentence in enumerate(sentences):
            new_sentence = L_translate(sentence, languages.ENGLISH, userdata.user_lang_in) #it's in english right?
            print(f"{bcolors.OKGREEN}{new_sentence}{bcolors.ENDC}")
            say(new_sentence, userdata.user_lang_in)
            #tts = gtts.gTTS(new_sentence, lang=userdata.user_lang_in)
            #tts.save("./hola.mp3")
            #if playsound_toggle:
            #    playsound("./hola.mp3")
            nb_translated[0] = nb_translated[0] + len(sentence)
            new_n_translated = new_n_translated + len(sentence)

        userdata.nb_translated_out = nb_translated
        return 'outcome1'

# define state InputRepeat
class InputRepeat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['new_story_in','user_lang_in','trgt_lang_in'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state InputRepeat')
        said_this = "You said this:"

        translator = Translator()

        new_said_this = L_translate(said_this, languages.ENGLISH, userdata.user_lang_in)
        print(f"{bcolors.OKCYAN}{new_said_this}{bcolors.ENDC}\n")
        say(new_said_this, userdata.user_lang_in)
        #tts = gtts.gTTS(new_said_this, lang=userdata.user_lang_in)
        #tts.save("./hola.mp3")
        #if playsound_toggle:
        #    playsound("./hola.mp3")
        if len(userdata.new_story_in)>2:
            print(f"{bcolors.OKBLUE}{userdata.new_story_in}{bcolors.ENDC}\n")
            say(userdata.new_story_in, userdata.trgt_lang_in)
            #tts = gtts.gTTS(userdata.new_story_in, lang=userdata.trgt_lang_in)
            #tts.save("./hola.mp3")
            #if playsound_toggle:
            #    playsound("./hola.mp3")

        return 'outcome1'


# define state UserConfirmation
class UserConfirmation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'],input_keys=['scenario_in','trgt_lang_in','usr_lang_in','usr_scenario_in','new_scenario_in'],output_keys=['scenario_out','new_scenario','usr_scenario_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state UserConfirmation')
        raw1 = "Please confirm with"
        tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
        raw2 = "cancel with"
        tl2 = L_translate(raw2, languages.ENGLISH, userdata.usr_lang_in)
        raw3 = "or repeat your previous action:"
        tl3 = L_translate(raw3, languages.ENGLISH, userdata.usr_lang_in)

        userinput = input(f"{bcolors.OKCYAN}{tl1} {bcolors.OKGREEN}[Enter]{bcolors.OKCYAN}, {tl2} {bcolors.OKGREEN}cancel{bcolors.OKCYAN} {tl3}:{bcolors.ENDC}\n")
        #print(f'{bcolors.OKCYAN}You entered {bcolors.ENDC}{bcolors.BOLD}{userinput}{bcolors.ENDC}')
        
        if userinput == "":
            raw1 = "Confirmed"
            tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
            print(f'{bcolors.OKCYAN}{tl1}.{bcolors.ENDC}\n')
            translator = Translator()
            story_to_translate = userdata.new_scenario_in
            translated_usr_story = L_translate(story_to_translate, userdata.trgt_lang_in, languages.ENGLISH)
            userdata.usr_scenario_out = userdata.usr_scenario_in + userdata.new_scenario_in
            userdata.scenario_out = userdata.scenario_in + translated_usr_story
            userdata.new_scenario = translated_usr_story
            analyze_text(story_to_translate,False, userdata.usr_lang_in)
            return 'outcome1'
        else:
            if userinput == "cancel":
                return 'outcome3'
            else:
                userdata.new_scenario = userinput
                return 'outcome2'


def analyze_text(text,verbose,user_language):
        #text = String(raw_text)
        #Separating into sentences, counting sentences
        sentences = list()
        remaining_text = text
        for iteration in range(50):
            dot_idx = remaining_text.find(".")
            qu_idx = remaining_text.find("?")
            ex_idx = remaining_text.find("!")
            idx_list = [dot_idx, qu_idx, ex_idx]
            for idx_idx, idx in enumerate(idx_list):
                if idx == -1 or idx < 4: #-1: not found, <4: sentence too short
                    idx_list[idx_idx] = 1000000
            min_idx = min(idx_list) + 1
            if min_idx >= 1000000:
                break
            sentences.append(remaining_text[:min_idx])
            remaining_text = remaining_text[min_idx:]
        sentence_nb = len(sentences)

        #Separating into words, counting words
        words = list()
        remaining_text = text
        for sentence in sentences:
            for iteration in range(10000):
                space_idx = sentence.find(" ")
                if space_idx == -1:
                    break
                if space_idx != 0:
                    words.append(sentence[:space_idx])
                sentence = sentence[space_idx+1:]
            words.append(sentence[:-1])
        word_nb = len(words)

        letter_nb = 0
        unique_words = 0
        for word_idx, word in enumerate(words):
            letter_nb = letter_nb + len(word)
            unique = True
            for other_word_idx in range(word_idx):
                #TODO: Improve word similarity test
                if word == words[other_word_idx]:
                    unique = False
                    break
            if unique:
                unique_words = unique_words + 1
            #if verbose:
            #    print(f"W:[{word}]")
        print(f"\n")

        if word_nb>0:
            avg_lett_nb_per100words = 100*letter_nb/word_nb
            avg_sentence_nb_per100words = 100*sentence_nb/word_nb
            #if verbose:
                #print(f"[DEBUG] number of characters: {letter_nb}\n")
                #print(f"[DEBUG] number of words: {word_nb}\n")
                #print(f"[DEBUG] number of sentences: {sentence_nb}\n")
                #print(f"[DEBUG] avg_lett_nb_per100words {avg_lett_nb_per100words}\n")
                #print(f"[DEBUG] avg_sentence_nb_per100words: {avg_sentence_nb_per100words}\n")
                #print(f"[DEBUG] Your wrote this (right?): {text}\n")
            A_value = 20
            B_value = 5
            W_value = 20
            coleman_liau_index = round(0.0588*avg_lett_nb_per100words - 0.296*avg_sentence_nb_per100words - 15.8, 1)
            raw1 = "Your language complexity index is "
            tl1 = L_translate(raw1, languages.ENGLISH, user_language)
            print(f"{bcolors.OKCYAN}{tl1} {bcolors.FAIL}{coleman_liau_index}{bcolors.OKCYAN}!{bcolors.ENDC}\n")
            if not verbose:
                #print(f"[DEBUG]: A_value (letter nb) is {0.0588*avg_lett_nb_per100words}.")
                if 0.0588*avg_lett_nb_per100words < A_value:
                    raw1 = "Tip: Try to use harder and longer words!"
                    tl1 = L_translate(raw1, languages.ENGLISH, user_language)
                    print(f"{tl1}")
                #print(f"[DEBUG]: B_value (letter nb) is {0.296*avg_sentence_nb_per100words}.")
                if 0.296*avg_sentence_nb_per100words > B_value:
                    raw1 = "Tip: Try to make longer sentences!"
                    tl1 = L_translate(raw1, languages.ENGLISH, user_language)
                    print(f"{tl1}")
                if word_nb < W_value:
                    raw1 = "Tip: Try to write more words!"
                    tl1 = L_translate(raw1, languages.ENGLISH, user_language)
                    print(f"{tl1}")
                if unique_words*2 < word_nb:
                    raw1 = "Tip: Try to write different words!"
                    tl1 = L_translate(raw1, languages.ENGLISH, user_language)
                    print(f"{tl1}")

        if verbose:
            raw1 = f"You wrote a total of {letter_nb} letters."
            tl1 = L_translate(raw1, languages.ENGLISH, user_language)
            raw2 = "You used a total of"
            tl2 = L_translate(raw2, languages.ENGLISH, user_language)
            raw3 = "different words!"
            tl3 = L_translate(raw3, languages.ENGLISH, user_language)
            raw4 = "Your listening score is "
            tl4 = L_translate(raw4, languages.ENGLISH, user_language)
            raw5 = "This is decreased when requesting transcription"
            tl5 = L_translate(raw5, languages.ENGLISH, user_language)
            
            print(f"{bcolors.OKCYAN}{tl1}{bcolors.ENDC}\n")
            print(f"{bcolors.OKCYAN}{tl2} {bcolors.FAIL}{unique_words}{bcolors.OKCYAN} {tl3}{bcolors.ENDC}\n")
            listening_score = round(1 - total_n_displayed/total_n_translated)
            print(f"{bcolors.OKCYAN}{tl4} {bcolors.FAIL}{listening_score}%{bcolors.OKCYAN} ({tl5}).\n")

# define state Evaluation
class Evaluation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['usr_story_in','trgt_lang_in','nb_translated_in','usr_lang_in','full_story_in'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Evaluation')
        analyze_text(userdata.usr_story_in, True, userdata.usr_lang_in)

        translation_ratio = math.floor(100*total_n_translated/total_n)
        raw1 = "The story has been"
        tl1 = L_translate(raw1, languages.ENGLISH, userdata.usr_lang_in)
        raw2 = "translated in your preferred language."
        tl2 = L_translate(raw2, languages.ENGLISH, userdata.usr_lang_in)
        print(f"{bcolors.OKCYAN}{tl1} {bcolors.FAIL}{translation_ratio}%{bcolors.OKCYAN} {tl2}.{bcolors.ENDC}\n")
        raw3 = "Thank you for your participation!"
        tl3 = L_translate(raw3, languages.ENGLISH, userdata.usr_lang_in)
        print(f"{bcolors.OKCYAN}{tl3}{bcolors.ENDC}")

        now = datetime.now()
        log_name = "log" + now.strftime("%Y%m%d%H%M%S")
        with open(log_name, 'w') as f:
            f.write("translation_ratio:\n")
            f.write(str(translation_ratio))
            listening_score = round(1 - total_n_displayed/total_n_translated)
            f.write("listening_score:\n")
            f.write(str(listening_score))
            f.write("usr_story:\n")
            f.write(userdata.usr_story_in)
            f.write("full_story:\n")
            f.write(userdata.full_story_in)

        return 'outcome1'

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['final_outcome', 'unexpected_outcome'])
    #state machine data
    logger = logging.getLogger('my-logger')
    logger.propagate = False
    sm.userdata.context = ""
    sm.userdata.debug = False
    sm.userdata.current_story = ""
    sm.userdata.new_ai_story = ""
    sm.userdata.new_user_story = ""
    sm.userdata.difficulty = 1 #0:easy, 1:normal, 2:advanced
    sm.userdata.user_language = languages.ENGLISH
    sm.userdata.target_language = languages.FRENCH
    #sm.userdata.target_language = languages.GERMAN
    sm.userdata.full_user_story = ""
    sm.userdata.n_char_translated = [0,0]
    sm.userdata.help_level = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialization', Initialization(), 
                               transitions={'outcome1':'StoryGeneration'},
                               remapping={'difficulty_out':'advanced_difficulty',
                                            'scenario_out':'new_user_story',
                                            'context_out':'context',
                                            'usr_lang_out':'user_language',
                                            'trg_lang_out':'target_language'})
        smach.StateMachine.add('StoryGeneration', StoryGeneration(), 
                               transitions={'outcome1':'StoryTTS'},
                               remapping={'scenario_in':'new_user_story',
                                            'context_in':'context',
                                            'new_scenario':'new_ai_story',
                                            'scenario_out':'current_story',
                                            'help_level_out':'help_level'})
        smach.StateMachine.add('StoryTTS', StoryTTS(), 
                               transitions={'outcome1':'StoryRecognition'},
                               remapping={'new_story_in':'new_ai_story',
                                            'user_lang_in':'user_language',
                                            'trgt_lang_in':'target_language',
                                            'difficulty_in':'advanced_difficulty',
                                            'nb_translated_in':'n_char_translated',
                                            'nb_translated_out':'n_char_translated'})
        smach.StateMachine.add('StoryRecognition', StoryRecognition(), 
                               transitions={'outcome1':'StoryHelp', 
                                            'outcome2':'InputRepeat', 
                                            'outcome3':'Evaluation',
                                            'outcome4':'StoryHelp2',
                                            'outcome5':'StoryRecognition'},
                               remapping={'scenario_in':'current_story',
                                            'trgt_lang_in':'target_language',
                                            'help_level_in':'help_level',
                                            'usr_lang_in':'user_language',
                                            'help_level_out':'help_level',
                                            'new_scenario':'new_user_story'})
        smach.StateMachine.add('StoryHelp', StoryHelp(), 
                               transitions={'outcome1':'StoryRecognition'},
                               remapping={'new_story_in':'new_ai_story',
                                            'difficulty_in':'advanced_difficulty',
                                            'trgt_lang_in':'target_language',
                                            'user_lang_in':'user_language',
                                            'nb_translated_in':'n_char_translated',
                                            'nb_translated_out':'n_char_translated'})
        smach.StateMachine.add('StoryHelp2', StoryHelp2(),
                                transitions={'outcome1':'StoryRecognition'},
                               remapping={'new_story_in':'new_ai_story',
                                            'trgt_lang_in':'target_language',
                                            'user_lang_in':'user_language',
                                            'nb_translated_in':'n_char_translated',
                                            'nb_translated_out':'n_char_translated'})
        smach.StateMachine.add('InputRepeat', InputRepeat(), 
                               transitions={'outcome1':'UserConfirmation'},
                               remapping={'new_story_in':'new_user_story',
                                            'user_lang_in':'user_language',
                                            'trgt_lang_in':'target_language'})
        smach.StateMachine.add('UserConfirmation', UserConfirmation(), 
                               transitions={'outcome1':'StoryGeneration', 
                                            'outcome2':'InputRepeat',
                                            'outcome3':'StoryRecognition'},
                               remapping={'scenario_in':'current_story',
                                            'usr_scenario_in':'full_user_story',
                                            'new_scenario_in':'new_user_story',
                                            'trgt_lang_in':'target_language',
                                            'usr_lang_in':'user_language',
                                            'scenario_out':'current_story',
                                            'new_scenario':'new_user_story',
                                            'usr_scenario_out':'full_user_story'})
        smach.StateMachine.add('Evaluation', Evaluation(), 
                               transitions={'outcome1':'final_outcome'},
                               remapping={'usr_story_in':'full_user_story',
                                            'trgt_lang_in':'target_language',
                                            'nb_translated_in':'n_char_translated',
                                            'usr_lang_in':'user_language',
                                            'full_story_in':'current_story'})
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
