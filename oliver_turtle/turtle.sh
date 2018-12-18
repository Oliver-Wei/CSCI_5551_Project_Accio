virtualenv ENV
source /home/turtlebot/ENV/bin/activate
pip install --upgrade google-cloud-speech
pip install pyserial
export GOOGLE_APPLICATION_CREDENTIALS="/home/turtlebot/Downloads/CSCI5551-speech-recognition-0f428c026c14.json"
