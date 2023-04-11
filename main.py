import cv2
import mediapipe as mp
import time
import matplotlib.pyplot as plt
from pyax12.connection import Connection
from simple_pid import PID

graph_x_video_displacement = []
graph_x_motor_displacement = []
graph_x_pid = []
graph_y_video_displacement = []
graph_y_motor_displacement = []
graph_y_pid = []
graph_time = []

# Port is to be changed depending on the port of the USB-Serial Board
# Baud rate is to be changed depending on the setting of the motor
serial_connection = Connection(port="COM6", baudrate=1000000)


# Tune PID values here. The setpoint is the center point of the video feed
pid_x = PID(0.036, 0, 0.0036, setpoint=320)
pid_y = PID(0.036, 0, 0.0036, setpoint=240)

# Connection test and initialization
print("Motor 1 Online: " + str(serial_connection.ping(1)))
serial_connection.goto(1, 0, 1023, True)
print("Motor 2 Online: " + str(serial_connection.ping(2)))
serial_connection.goto(2, 0, 1023, True)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


def findPosition(image, draw=True):
    lmList = []
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        for id, lm in enumerate(results.pose_landmarks.landmark):
            h, w, c = image.shape
            cx, cy = int(lm.x * w), int(lm.y * h)
            lmList.append([id, cx, cy])
            #cv2.circle(image, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
    return lmList


print("Loading Video Feed")
cap = cv2.VideoCapture(0)
print("Video Feed Loaded")
start_time = time.time()
with mp_pose.Pose(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7,) as pose:
    while cap.isOpened():
        success, image = cap.read()
        image = cv2.resize(image, (640,480))
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue
            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        results = pose.process(image)
        # Draw the pose annotation on the image.
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        lmList = findPosition(image, draw=True)
        cv2.imshow('MediaPipe Pose', image)
        try:
            # Actions done if nose is detected
            if lmList[0][1]:
                # Print time of cycle since start time and append for graphing
                print(time.time() - start_time)
                graph_time.append(time.time() - start_time)
                # Print current video and motor displacement values and append for graphing
                print(str(lmList[0][1]) + "," + str(lmList[0][2]))
                print(str(320 - lmList[0][1]) + "," + str(240 - lmList[0][2]))
                graph_x_video_displacement.append(320-lmList[0][1])
                graph_x_motor_displacement.append(serial_connection.get_present_position(1, True))
                graph_y_video_displacement.append(240 - lmList[0][1])
                graph_y_motor_displacement.append(serial_connection.get_present_position(2, True))
                # Calculate motor movement based on PID values and graph and print the movement numbers
                control_x = -pid_x(lmList[0][1])
                graph_x_pid.append(control_x)
                control_y = -pid_y(lmList[0][2])
                graph_y_pid.append(control_y)
                print(str(control_x) + ", " + str(control_y))
                print("---------------------------")
                # Moves motor
                serial_connection.goto(1, int(round(control_x) + serial_connection.get_present_position(1, True)),
                                       1023, True)
                serial_connection.goto(2, int(round(control_y) + serial_connection.get_present_position(2, True)),
                                       1023, True)
        except:
            #Error handling
            print("Out of Frame")
        # Button to exit and go graph
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# After the loop release the cap object
cap.release()
# Destroy all the windows
cv2.destroyAllWindows()
serial_connection.close()
# Creates graph for tuning purposes
figure, axis = plt.subplots(2, 2)
axis[0, 0].plot(graph_time, graph_x_pid)
axis[0, 0].set_title("PID X and X Motor Displacement")
axis[0, 0].plot(graph_time, graph_x_motor_displacement)
axis[1, 0].plot(graph_time, graph_x_video_displacement)
axis[1, 0].set_title("X Video Displacement")
axis[0, 1].plot(graph_time, graph_y_pid)
axis[0, 1].set_title("PID Y and Y Motor Displacement")
axis[0, 1].plot(graph_time, graph_y_motor_displacement)
axis[1, 1].plot(graph_time, graph_y_video_displacement)
axis[1, 1].set_title("Y Video Displacement")
plt.legend()
plt.show()
