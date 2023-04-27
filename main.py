from djitellopy import tello
import cv2
from pupil_apriltags import Detector
import tellocontrol as control
import math
import keyboard


# Create the AprilTag detector for the 36h11 family
at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)


def distance_calculator(tag):
    one_two_length = math.sqrt(
        math.pow((tag.corners[0][0] - tag.corners[1][0]), 2) + math.pow((tag.corners[0][1] - tag.corners[1][1]), 2))
    two_three_length = math.sqrt(
        math.pow((tag.corners[1][0] - tag.corners[2][0]), 2) + math.pow((tag.corners[1][1] - tag.corners[2][1]), 2))
    three_four_length = math.sqrt(
        math.pow((tag.corners[2][0] - tag.corners[3][0]), 2) + math.pow((tag.corners[2][1] - tag.corners[3][1]), 2))
    four_one_length = math.sqrt(
        math.pow((tag.corners[3][0] - tag.corners[0][0]), 2) + math.pow((tag.corners[3][1] - tag.corners[0][1]), 2))
    avg_length = (one_two_length + two_three_length + three_four_length + four_one_length) / 4
    dist = 5703 * pow(avg_length, -1.02)
    return dist


drone = tello.Tello()
drone.connect()
print(drone.get_battery())
drone.streamon()
drone.takeoff()
lr_error = 0
ud_error = 0
fb_error = 0
y_error = 0

while True:
    image = drone.get_frame_read().frame
    if keyboard.is_pressed("q"):
        print("q pressed")
        drone.land()
    debug_image = image
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(image, estimate_tag_pose=False, camera_params=None, tag_size=None)

    if not tags:
        drone.send_rc_control(0, 0, 0, 0)

    targets = [0, 1, 2]

    for tag in tags:
        if targets.__contains__(tag.tag_id):
            if keyboard.is_pressed("q"):
                print("q pressed")
                drone.land()

            dist = distance_calculator(tag)
            print(dist)

            # k = [L,R,U,D,F,B, y, y]
            # kp = [0.1, 0.1, 0.6, 0.9, 0.1, 0.1, 0.2, 0.1]
            # kd = [2, 2, 3, 3, 2, 2, 1, 1]
            # [FB, UD, Y]
            kp = [0.5, 0.14, 0.15]
            kd = [3, 2, 1.5]

            old_lr = lr_error
            old_ud = ud_error
            old_fb = fb_error
            old_y = y_error
            lr_error = tag.center[0] - 640
            ud_error = (tag.center[1] - 360) * -1
            fb_error = dist - 70
            y_error = tag.center[0] - 640

            # if lr_error > 0: #left
            #     LR = int(kp[0] * lr_error + kd[0] * (lr_error - old_lr))
            # else: #right
            #     LR = int(kp[0] * lr_error + kd[0] * (lr_error - old_lr))

            if fb_error < 0: #back
                FB = int(kp[0]*fb_error + kd[0]*(fb_error - old_fb))
            else: #forward
                FB = int(kp[0] * fb_error + kd[0] * (fb_error - old_fb))

            if ud_error > 0: #up?
                UD = int(kp[1] * ud_error + kd[1] * (ud_error - old_ud))
            else: #down?
                UD = int(kp[1] * ud_error + kd[1] * (ud_error - old_ud))

            if y_error > 0: #turn to the right
                print("turn right")
                Y = int(kp[2] * y_error + kd[2] * (y_error - old_y))
            else: #turn to the left
                Y = int(kp[2] * y_error + kd[2] * (y_error - old_y))

            #drone.send_rc_control(LR, FB, UD, Y)
            drone.send_rc_control(0, FB, UD, Y)

    cv2.waitKey(1)
    cv2.imshow('Test 2', debug_image)