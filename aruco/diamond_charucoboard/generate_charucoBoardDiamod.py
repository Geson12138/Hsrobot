import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

img = cv2.aruco.drawCharucoDiamond(dictionary=aruco_dict, ids=(0, 1, 2, 3), squareLength=200, markerLength=150, marginSize=40, borderBits=1)
cv2.imwrite("test_charucoboarddiamond.jpg", img)

cv2.imshow('Charuco Diamond', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
