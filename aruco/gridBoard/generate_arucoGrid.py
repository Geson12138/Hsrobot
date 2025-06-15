import cv2
import cv2.aruco as aruco

# Create gridboard, which is a set of Aruco markers
# the following call gets a board of markers 5 wide X 7 tall
gridboard = aruco.GridBoard(
        size=(3,4), 
        markerLength=0.1, 
        markerSeparation=0.02, 
        dictionary=aruco.getPredefinedDictionary(aruco.DICT_6X6_250))

# Create an image from the gridboard
img = aruco.drawPlanarBoard(board=gridboard,outSize=(360, 480),marginSize=10,borderBits=1) # Draw the gridboard, unit: pixel
cv2.imwrite("test_gridboard.jpg", img)

# Display the image to us
cv2.imshow('Gridboard', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()