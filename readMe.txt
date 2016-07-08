Keep .cpp and .h files in same folder

Input images are not required to be kept in same folder but path to input image need to be provided to the program

command for compiling the program
g++ -o curveToFlat curveToFlat.cpp $(pkg-config opencv --cflags --libs)


How to run

./curveToFlat <path/to/image> <1: to turn on debugMode, 0 otherwise> <1: to look for boundary, 0 otherwise>

if boundary are used then spots will not be detected

if debugMode is turned on then intermediate images will be shown as well as saved as jpg files in the program directory


Error messages:

1) If 4 corners not found: "4 corners are not found ... quitting"
2) If bounday is not found: "Color boundary contour not found ... quitting"
3) If inner circle with black boundary is not found: "no circle found ... quitting!"

