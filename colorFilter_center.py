import cv2
import numpy as np
import math

_SHOW_IMAGE = True

class StopSignNav():

    def __init__(self):
        print("initilizing Stop-Sign Finder")


    def find_stopsign(self, frame):

        show_image("stop sign", frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        # res = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("frame", mask)
        cv2.waitKey(0)

        # calculate moments of binary frame
        M = cv2.moments(mask)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # put text and highlight the center

        cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
        cv2.putText(frame, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        show_image("stop sign with centroid" , frame)

        return (cX,cY)

    def calculate_steering(self, centroid):
        pass


        # height, width, channels = frame.size

        #  x_pixel_diff =




def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)
        cv2.waitKey(0)


def video_capture():
    cap = cv2.VideoCapture(0)

    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        # calculate moments of binary frame
        M = cv2.moments(mask)

        # calculate x,y coordinate of center
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except ZeroDivisionError as e:
            print(e)
            continue

        # put text and highlight the center

        cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
        cv2.putText(frame, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        def calculate_heading(centroid):
            # calculate heading
            height, width, _ = frame.shape

            camera_mid_offset_percent = 0.00  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
            mid = int((width / 2 )* (1 + camera_mid_offset_percent))

            # calculate centroid offset from mid
            x_offset = cX - mid ; y_offset = height - cY

            angle_to_mid_radian = math.atan(x_offset / y_offset)

            print('angle heading: %s' % (np.rad2deg(angle_to_mid_radian)))

            cv2.line(frame, (mid , height), (centroid[0], centroid[1]), (0,255,0), 2)

            return angle_to_mid_radian

        def caluclate_speed():
            '''
            do steps at a time
            speed = value
            publish speed
            sleep 1 sec
            speed = 0
            publish


            if depth to centroid is < threshhold : return: successful
            else:  continue
            '''
            pass







        calculate_heading((cX,cY))


        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    video_capture()
    #StopSignObj = StopSignNav()

    #frame = cv2.imread('Stop-Sign-Web.jpg')
    #print(StopSignObj.find_stopsign(frame))

