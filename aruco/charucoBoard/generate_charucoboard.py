import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

gridboard = cv2.aruco.CharucoBoard(
    size=(4, 4), 
    squareLength= 0.015, 
    markerLength= 0.011, 
    dictionary=aruco_dict)

# Create an image from the gridboard
img = cv2.aruco.drawPlanarBoard(board=gridboard,outSize=(640, 480),marginSize=20,borderBits=1) # Draw the gridboard, unit: pixel
cv2.imwrite("test_charucoboard.jpg", img)

# Display the image to us
cv2.imshow('charucoboard', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()
