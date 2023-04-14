import cv2
import numpy as np

real_width = 12.
real_height = 8.

img = cv2.imread(
    "/home/ninefish/nine-fish/JsonFileBased_RobotDecision/tools/resource/RMUL.png")

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%f,%f" % (float(
            x)/float(img.shape[1]) * real_width, float(y)/float(img.shape[0]) * real_height)
        print(xy)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 255), thickness=1)
        cv2.imshow("image", img)


cv2.namedWindow("image", cv2.WINDOW_GUI_NORMAL)
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image", img)
while (True):
    try:
        cv2.waitKey(100)
    except Exception:
        cv2.destroyWindow("image")
        break
