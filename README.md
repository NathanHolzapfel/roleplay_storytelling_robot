# HOW TO INSTALL AND RUN:
## Start a ros noetic docker  [Only once]
You may first need to download and install Docker (https://docs.docker.com/get-docker/) and its ROS containers (http://wiki.ros.org/docker/Tutorials/Docker). Installing Docker Desktop isn't necessary.
```
sudo docker run -it --network="host" ros:noetic-robot
sudo docker ps -a
```
Find new docker's name, we assume its name is 'magical_mcclintock' from now on
```
sudo docker exec -it magical_mcclintock bash
```
## Install project package  [Only once]
After entering docker container
```
cd ~/catkin_ws/src
git clone https://github.com/NathanHolzapfel/roleplay_storytelling.git
cd roleplay_storytelling
chmod +x scripts/roleplayStorytellingMain.py
chmod +x scripts/roleplaySay.py
```

## Optional: Disable log messages in Smach
In '/opt/ros/noetic/lib/python3/dist-packages/smach': comment print lines and add return statements
```
vim log.py
```
## Update project package  [When project scripts have been modified]
Go into scripts folder
```
git stash
git pull
chmod +x scripts/roleplayStorytellingMain.py
chmod +x scripts/roleplaySay.py
```
## Install necessary modules [Only once]
```
sudo  apt-get install pip
pip install requests
pip install helpers
pip install googletrans
pip install gtts
pip install playsound
pip install SpeechRecognition
pip install deepl
```
You may also have to install QT python interface modules.

## Install KoboldAI [Only once]
(choose the place where to install)
```
git clone https://github.com/koboldai/koboldai-client
sudo apt-get install wget
```
Go into new folder
```
./play.sh
```
## Set up QT connection  [Only once, unless network settings are changed]
In ~:
```
git clone https://github.com/luxai-qtrobot/software.git
```
Go into 'software' folder
```
cp -r headers/* ~/catkin_ws/devel/
```
Connect to QT Wifi (name:QTRD000109, pwd:11111111) and to Ethernet cable
```
ifconfig
```
Find your ID (192.168.4.158 from now on)
```
sudo apt-get install vim
```
In ~:
```
vim .bash_aliases
```
Add (press i to edit):
```
 ## QTrobot
 export ROS_IP=192.168.4.158
 export ROS_MASTER_URI=http://192.168.4.1:11311
```
If this does not work, the IPs may differ. The ROS_IP can be found with ifconfig, as to the other,
try the mentioned APs in the documents, like this one: https://docs.google.com/presentation/d/1iQWH3Jt66yaCi9f5TknXYQX5uQsQUJFEkcyVOa_SG9E/edit#slide=id.ge075ed31f5_1_0
Save by pressing Esc and then :wq! and finally Enter
Exit and execute docker again and test:
```
rostopic list
```
## Restart a docker image named magical_mcclintock  [When docker was stopped]
```
sudo docker ps -a
sudo docker restart magical_mcclintock
sudo docker exec -it magical_mcclintock bash
```
## Set up the ros docker image [When opening a new docker terminal]
```
source ros_entrypoint.sh
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
```
## Using KoboldAI [Every new session]
Go into koboldAI folder
```
./play.sh
```
Right-click on network link and select Open link, which should open the interface on host's browser. An alternative is to use a Google Colab to run better models. In that case, the address for the story generation request in the code should be updated with the browser link to the tunnel interface.

## Run project package [Every time]
```
rosrun roleplay_storytelling roleplaySay.py
```
In a new terminal:
```
rosrun roleplay_storytelling roleplayStorytellingMain.py
```

# ARCHITECTURE:

## Files

- "launch" folder: initially created to run both modules at once, this initially failed and was not pursued further.
- "msg" and "srv" folder: These folders contain simple definitions, required for the ROS communication between the modules.
- "CMakeLists" and "package": Required default components for the ROS compilation/execution.
- "scripts" folder: The most important folder, where can be found the actual programs (scripts). They are to be run separately and communicate with ROS messages as defined in "msg".
 - "roleplaySay.py": This script communicates the relevant command to the robot. Modify this script if the relevant command has changed, typically because of a robot change.
 - "roleplayStorytellingMain.py": This script handles the core tasks: queries to the user, communications and data handling for translations and text generation, score computing, etc.

## Main script
The main program (roleplayStorytellingMain.py) uses at its core the Smach state machine. As such, it declares smach classes for each state (like "class Initialization(smach.State)"). Some of their functions have been declared at the start of the file. At the end of the file however, lies the definition of the "main()" function, where the state machine is further defined (look up smach explanations and tutorials if necessary to understand). Here are what each state does:
 - Initialization: This step allows the user to choose their languages (overwriting the program's default), the initial difficulty and the initial context. The initial contexts are at present hard-coded, with a choice among 4 of them.
 - StoryGeneration: This step uses text generation to provide the robot's contribution to the story. The generation is performed in english, and the context is saved in 10 memory slots so that the algorithm can stay relevant to what happened previously without being limited by old statements.
 - StoryTTS: This step communicates the robot's contribution to the user. It performs translation and sends ROS messages for vocal output. Due to the necessity of translating the text sentence by sentence depending on difficulty, this step may not be as straightforward as may be expected.
 - StoryRecognition: This step asks the user for their input. It also contains a prototype for speech recognition. It is not presently reliable (and is consequently unused), one reason being the low quality of the robot's microphone.
 - StoryHelp: Similar to StoryTTS, this step communicates the robot's contribution to the user but with text too.
 - StoryHelp2: This time, the text is 100% translated to make it even easier.
 - InputRepeat and UserConfirmation: A confirmation to the user about their contribution.
 - Evaluation: The final part of the algorithm, where it analyzes the text, provides metrics, and thanks the user for their participation.
