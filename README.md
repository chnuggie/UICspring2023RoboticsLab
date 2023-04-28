# UICspring2023RoboticsLab
# This code allows a Tello drone to follow an april tag at a distance of 70cm. It is calibrated with an april tag of size 5.5cm. 
# The code will still work with an april tag of a different size but the distance calculation will not be accurate.
# In order to run the code pip-install can be used to import all needed libraries, the libraries neede are listed at the top of the main file
# How to run the code:
#     - turn on tello drone, connect computer to tello drone's wifi
#     - run the code
#            - the drone will take off and hover until it sees an april tag
#            - Ensure that the april tag is in the drone's field of view
#            - presssing "q" on your keyboard will land the drone, then simply stop running the code
#  How the code works:
#      - detector function: this function is from the apriltag library and detects the prescence of an april tag
#      - distance calculator function: uses the size of the apriltag's edges in pixels to calculate size
#      - main loop: uses openCV to open and view feed of camera, uses PD loop to control velocity of drone with drone functions.

#Tello API Documentation: https://djitellopy.readthedocs.io/en/latest/tello/
# April Tag Documentation: https://pupil-apriltags.readthedocs.io/en/stable/api.html
