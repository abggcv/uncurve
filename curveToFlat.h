#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
 
using namespace cv;
using namespace std;

bool debugMode = false;
bool isBoundary = false;
bool isResize = false;

double reszFactor = 1.0;

Point centerBlackCircle;

//string configFile = "config.txt";

//range of HSV values for red color
Scalar low1(0, 70, 100);
Scalar high1(10, 255, 255);

Scalar low2(170, 70, 100);
Scalar high2(180, 255, 255);

