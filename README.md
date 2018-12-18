# CSCI_5551_Project_Accio
In this project, Turtlebot and Baxter helps a person fetch something mainly based on speech recognition, object detection and inverse kinematics of Baxter.

Turtlebot

Due to some environment problems, three separate programs have to run in the computer connected to the turtlebot. (We will try to solve the problem in further work.)

	1. Speech recognition
		
	virtualenv ENV

	source /home/turtlebot/ENV/bin/activate

	pip install --upgrade google-cloud-speech

	pip install pyaudio

	export GOOGLE_APPLICATION_CREDENTIALS="path/xx.json"

	python ~/oliver_turtle/audio_test.py

	2. Move of turtlebot

	roslaunch turtlebot_bringup minimal.launch

	In another terminal: python ~/oliver_turtle/turtle.py

	3. Send the request to Baxter

	python ~/oliver_turtle/transmit_test.py

  Baxter
  
  Due to some environment problems, two separate programs have to run in the computer connected to the turtlebot. (We will try to solve   the problem in further work.)

	1. Object detection
	
	virtualenv --system-site-packages -p python3 ./venv
	
	source ./venv/bin/activate
	
	pip install --upgrade pip
	
	pip install --upgrade tensorflow
	
	pip install opencv-python
	
	pip install Pillow
	
	cd ~/models/research
	
	export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
	
	python object_detection/object_detection.py


	2. Baxter Manipulation

	python ~/oliver_baxter/baxter.py
