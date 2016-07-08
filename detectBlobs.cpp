#include "curveToFlat.h"


void showReducedImage(string title, Mat img, double f)
{
	Mat temp;
	resize(img, temp, Size(), f, f, CV_INTER_AREA);

	imshow(title, temp);	
}


void printInDebugMode(string mssg)
{
	if(debugMode)		
		cout << mssg << endl;
}



void detectBlobs(Mat img, vector<Point> &circlecenters, vector<Point> &squarecenters){

	Mat frame = Mat::zeros(img.size(), CV_8UC3);


	Mat gray; Mat threshold_output;

	if(img.channels() == 3){
		cvtColor(img, gray, CV_BGR2GRAY);
	
	
	GaussianBlur(gray, gray, Size(31, 31), 0, 0);
	
	
	gray.copyTo(threshold_output);
	
	printInDebugMode("adaptive threshold");
	
	adaptiveThreshold(threshold_output, threshold_output, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 51, 0);	
	}

	if(img.channels() == 1)
		img.copyTo(threshold_output);

	int grad_thresh_min = 100;
	int grad_thresh_max = 300;
	double circleError = 0.2;
	double areaError = 0.15;
	int minCircleSize = 10;
	
	Canny(threshold_output, threshold_output, grad_thresh_min, grad_thresh_max, 3); // Creates Edges
	
	printInDebugMode("canny done");


	//find contours
	vector<vector<Point> > contoursAll;
	vector<Vec4i> hierarchy;
	findContours(threshold_output.clone(), contoursAll, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0,0));
	
	stringstream ss;

	ss << "found countours: " << contoursAll.size();

	printInDebugMode(ss.str());

	ss.str(string());

	//Mat to draw non-noisy contours
	Mat cntImg = Mat::zeros(img.size(), CV_8UC3);

	//we have edges and contours and ready to detect circles or squares in blobs
	int totalPotentialCircles = 0;

	int line_thresh = (int)img.cols*0.03;

	vector<int> indexDetectedCircles, indexDetectedSquares;

	printInDebugMode("checking contours for circles and squares");

	if (contoursAll.size() > 0) {
		printInDebugMode("contours present");
		vector<Point> contours;
		int cntCount = 0;
		contours = contoursAll[cntCount];
		//approxPolyDP(Mat(contours), approx, 2, true);

		//int line_thresh = 3;  //number of approx. contour points for a line, triangle

		while (cntCount < contoursAll.size()-1) {
			/*ss << "processing contour: " << cntCount;
			printInDebugMode(ss.str());
			ss.str(string());*/
			if(contours.empty()){
				ss << "skipping countour: " << cntCount;
				printInDebugMode(ss.str());
				ss.str(string());
				cntCount++;
				contours = contoursAll[cntCount];
				continue;
			}

			RotatedRect box;
								
			if ((contours.size() > (line_thresh))) {  // line_thresh is camerawidth*0.03 to filter small noise contours if any

				double area = abs(contourArea(contours));

				if(area > 0.00007*img.rows*img.cols && area < 0.2*img.rows*img.cols)
					drawContours(cntImg, contoursAll, cntCount, Scalar(0, 255, 255), 2);
				else{
					cntCount++;
					contours = contoursAll[cntCount];
					continue;
				}

				box = fitEllipse(contours);
					
				Size size((int) (box.size.width * 0.5), (int) (box.size.height * 0.5));
				double smallAxis, longAxis;

				if (size.width < size.height) {
					smallAxis = box.size.width * 0.5;
					longAxis = box.size.height * 0.5;
				} else {
					smallAxis = box.size.height * 0.5;
					longAxis = box.size.width * 0.5;
				}                  				
				
				double boxArea = box.size.width*box.size.height;

				// validate as square and draw green contour around

				if (area >= 0.70*boxArea && area <=  0.90*boxArea) {
					//mark as square
					drawContours(frame, contoursAll, cntCount, Scalar(0, 0, 255), 4);
					indexDetectedSquares.push_back(cntCount);
					squarecenters.push_back(box.center);
					//printInDebugMode("Found square");
				}


				// validate as circle and draw red boundary around
				double circleMeasure = (longAxis - smallAxis) / longAxis;
				double radius = (longAxis + smallAxis) / 2;
				double perimeter = arcLength(contours, true);
				/*if((perimeter > maxPeri) && (duplicate != null)){
					maxPeri = perimeter;
					//cvDrawContours(duplicate, contours, CvScalar.WHITE, CvScalar.WHITE, -1, 2, CV_AA);
				}*/

				//double area = contourArea(contours, CV_WHOLE_SEQ, 0));
				double area_diff = abs(area - ((radius * perimeter) / 2));
				double area_delta = area_diff / area;

				if (circleMeasure < circleError && smallAxis >= minCircleSize && area_delta < areaError) {
					// add to list of potential circles
					/*double rFromArea = sqrt(area / CV_PI);
					double rFromPeri = (perimeter / (2 * CV_PI));
					//Circle c = new Circle(box.center().x(), box.center().y(), radius, longAxis, smallAxis, label_size);
					//c.contour = new CvContour(contours.ptr());
					//double carea = Math.abs(cvContourArea(contours));
					
					//c.area = longAxis*longAxis;
					if (Biggest == null) {
						Biggest = c;
					} else if (c.radius > Biggest.radius) {
						Biggest =c;							
					}
					potential_circles.add(c);*/
					indexDetectedCircles.push_back(cntCount);
					drawContours(frame, contoursAll, cntCount, Scalar(0, 255, 0), 3);
					circlecenters.push_back(box.center);
					//printInDebugMode("Found circle");
				}
			}

			cntCount++;
			contours = contoursAll[cntCount];
		}
	}
	
	ss << "No of squares found: " << indexDetectedSquares.size();

	printInDebugMode(ss.str());

	ss.str(string());

	ss << "No of circles found: " << indexDetectedCircles.size();

	printInDebugMode(ss.str());

	ss.str(string());

	showReducedImage("blobs", frame, reszFactor);
	showReducedImage("all contours", cntImg, reszFactor);
	waitKey(0);	
	
}


void extractColor(Mat img, Mat &colorMat)
{
	//convert to hsv
	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	
	Mat1b mask1, mask2;
    	inRange(hsv, low1, high1, mask1);
    	inRange(hsv, low2, high2, mask2);

	//Mat1b 
	colorMat = mask1 | mask2;

	//morphological operations
	dilate(colorMat, colorMat, Mat());
	erode(colorMat, colorMat, Mat());
	morphologyEx(colorMat,colorMat,MORPH_CLOSE,getStructuringElement( MORPH_ELLIPSE,Size(7,7)));
	
	if(debugMode){
		showReducedImage("color", colorMat, 0.2);
		//imwrite("color.jpg", colorMat);
		waitKey(0);
	}

}




int main( int argc, char** argv)
{
	//debugMode
	if(argc > 2){
		if(strcmp(argv[2],"1")==0)
			debugMode = true;
	}

	//resize factor
	if(argc > 3)
		reszFactor = atof(argv[3]);

	//read image	
	Mat input = imread(argv[1]);

	if(input.empty()){
		cout << "Check input image" << endl;
		return -1;
	}

	vector<Point> circleCenters, squareCenters;
	
	//find circle centers
	detectBlobs(input, circleCenters, squareCenters);

	cout << "no of circle centers: " << circleCenters.size() << endl;
	cout << "no of square centers: " << squareCenters.size() << endl;

	circleCenters.clear();
	squareCenters.clear();


	Mat color;
	extractColor(input, color);	

	//find square centers		
	detectBlobs(color, circleCenters, squareCenters);

	cout << "no of circle centers: " << circleCenters.size() << endl;
	cout << "no of square centers: " << squareCenters.size() << endl;


	return 1;

}

