#include "curveToFlat.h"

bool myComparison(const pair<int,int> &a,const pair<int,int> &b)
{
       return a.first > b.first;
}

struct sort_pred {
    bool operator()(const std::pair<double,int> &left, const std::pair<double,int> &right) {
        return left.first < right.first;
    }
};



void showReducedImage(string title, Mat img, double f)
{
	Mat temp;
	resize(img, temp, Size(), f, f, CV_INTER_AREA);

	imshow(title, temp);	
}



void findCornersByBoundary(vector<Point> cnt, vector<Point> &cornerPts, vector<int> &cornerInd, double threshAngle)
{
	Point2f p1, p2, p3;

	int ind = 0;

	for(int i = 0; i < cnt.size(); i++)
	{
		
		p1 = cnt[i];

		if(i == cnt.size()-2)
		{
			ind = i+1;
			p2 = cnt[i+1];
			p3 = cnt[0];

			//cout << "p1, p2, p3: " << i+1 << ", " << i+2 << ", " << 1 << endl;
		}
		
		else if(i == cnt.size()-1)
		{
			ind = 0;
			p2 = cnt[0];
			p3 = cnt[1];

			//cout << "p1, p2, p3: " << i+1 << ", " << 1 << ", " << 2 << endl;
		}

		else
		{
			ind = i+1;
			p2 = cnt[i+1];
			p3 = cnt[i+2];

			//cout << "p1, p2, p3: " << i+1 << ", " << i+2 << ", " << i+3 << endl;
		}


		//vectors P12 and P23 -- |p1-p2| and |p2-p3|
		Point2f P12, P32;

		P12 = Point2f(p1.x - p2.x, p1.y - p2.y);
		P32 = Point2f(p3.x - p2.x, p3.y - p2.y);

		double dot = P12.x*P32.x + P12.y*P32.y;      //dot product
		double det = P12.x*P32.y - P12.y*P32.x;      // determinant
		double angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)

		//cout << "angle: " << angle << endl;

		if(abs(angle) < threshAngle)
		{
			cornerInd.push_back(ind);
			cornerPts.push_back(p2);
		}
	}

}

bool findCornersBySquares(Mat colorMat, vector<Point> &cornerPts)
{
	//resize
	Mat im1, im2;
	if(isResize)
	{	
		resize(colorMat, im1, Size(), 0.2, 0.2, CV_INTER_AREA);	//0.2 for detecting just circles
		resize(colorMat, im2, Size(), 0.1, 0.1, CV_INTER_AREA); // 0.1 for detecting squares
	}
	else
	{
		colorMat.copyTo(im1);
		colorMat.copyTo(im2);
	}

	//cout << "size of im: " << im.size() << endl;

	
	//convert to gray scale
	Mat gray1, gray2;
	cvtColor(im1, gray1, CV_BGR2GRAY);
	cvtColor(im2, gray2, CV_BGR2GRAY);
	
	/*if(debugMode){
		// Show gray image
		imshow("gray", gray );
		waitKey(0);	
	}*/


	// Set up the detector with default parameters.
	SimpleBlobDetector::Params params1, params2;
	
	// Filters for blobs
	// Change thresholds for circular detection
	params1.minThreshold = 0;
	params1.maxThreshold = 255;
	params1.thresholdStep = 1;
	params1.minDistBetweenBlobs = 1;
	params1.filterByCircularity = true;
	params1.minCircularity = 0.9;   //0.90 for circulars  //0.6 - 0.8 for squares
	params1.maxCircularity = 1.0;   //0.99 for circulars

	params2.minThreshold = 0;
	params2.maxThreshold = 255;
	params2.thresholdStep = 1;
	params2.minDistBetweenBlobs = 1;
	params2.filterByCircularity = true;
	params2.minCircularity = 0.6;   //0.90 for circulars  //0.6 - 0.8 for squares
	params2.maxCircularity = 0.8;   //0.99 for circulars
		
	SimpleBlobDetector detector1(params1);
	SimpleBlobDetector detector2(params2);
	
	cout << "finding blobs" << endl;

	vector<KeyPoint> keypoints1, keypoints2;
	detector1.detect( gray1, keypoints1);
	detector2.detect( gray2, keypoints2);
 
	if(keypoints2.size() < 4)
	{
		cout << "Can not detect 4 corners" << endl;
		return false;
	}

	cout << "circular blobs detected: " << keypoints1.size() << endl;
	cout << "rectangular blobs detected: " << keypoints2.size() << endl;
	
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints1, im_with_keypoints2;
	drawKeypoints( im1, keypoints1, im_with_keypoints1, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	drawKeypoints( im2, keypoints2, im_with_keypoints2, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 	
	cout << "find centroids of blobs" << endl;	

	//find centeroid of all keypoints
	Point cent(0, 0);	

	for(size_t i = 0; i < keypoints1.size(); i++)
	{
		cent.x += (int)keypoints1[i].pt.x;
		cent.y += (int)keypoints1[i].pt.y;		
	}

	cent.x /= (2*keypoints1.size());
	cent.y /= (2*keypoints1.size());

	cout << "center point: " << cent << endl;
	
	vector<pair<double, int> > distFromCenter;
	for(size_t i = 0; i < keypoints2.size();i++)
	{
		Point2f p((int)keypoints2[i].pt.x, (int)keypoints2[i].pt.y);
		
		double dist = (p.x-cent.x)*(p.x-cent.x)+(p.y-cent.y)*(p.y-cent.y);		
		
		if(dist < 10)		
			continue;

		distFromCenter.push_back(make_pair(dist,i));
		
	}

	//sort distances
	sort(distFromCenter.begin(), distFromCenter.end(), sort_pred());
	
	//draw circle for first 4
	for(size_t i = 0; i < 4; i++){
		circle(im_with_keypoints2, keypoints2[distFromCenter[i].second].pt, 3, Scalar(255, 0, 0));
		cornerPts.push_back(Point((int)(keypoints2[distFromCenter[i].second].pt.x/0.1), (int)(keypoints2[distFromCenter[i].second].pt.y/0.1)));	
	}

	if(debugMode){
		// Show blobs
		showReducedImage("circular", im_with_keypoints1, 1.0);
		showReducedImage("rectangular", im_with_keypoints2, 1.0);
		waitKey(0);	
	}
	
	return true;
}



bool detectCircle(Mat img)
{
	//apply padding border around image
	Mat image = Mat::zeros((int)(1.6*img.rows), (int)(1.6*img.cols), CV_8UC3);

	Rect imgRect = Rect(0, (int)(0.3*img.rows), img.cols, img.rows);

	img.copyTo(image(imgRect));

	if(debugMode){
		showReducedImage("padded", image, 0.2);
		imwrite("padded.jpg", image);
		waitKey(0);
	}

	Mat hsv;
	cvtColor(image.clone(), hsv, CV_BGR2HSV);

	Mat thresh;
	inRange(hsv, Scalar(0, 0, 0, 0), Scalar(180, 255, 80, 0), thresh);  //detect black color

	if(debugMode){
		showReducedImage("threshold", thresh, 0.2);
	//imshow("threshold", thresh);
		waitKey(0);
	}

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours( thresh.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//draw contours
	Mat cntimg = Mat::zeros(image.rows, image.cols, CV_8UC3);

	int ind = -1;

	for(int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);

		if(area > 0.7*img.rows*img.cols)
			continue;

		if(area < 0.1*img.rows*img.cols)
			continue;

		drawContours(cntimg, contours, i, Scalar(255, 255, 0));		
		
		ind = i;
	}

	if(ind < 0)
	{
		cout << "no circle found ... quitting!" << endl;
		return false;
	}
	

	Rect R = boundingRect(contours[ind]);

	Point2f center;
	float radius;
	minEnclosingCircle( (Mat)contours[ind], center, radius);

	rectangle(cntimg, R, Scalar(255, 0, 255));

	circle(cntimg, center, radius, Scalar(0, 255, 255));

	vector<Point2f> src_pts;
	vector<Point2f> dst_pts;

	src_pts.push_back(Point(R.x,R.y));
	src_pts.push_back(Point(R.x+R.width,R.y));
	src_pts.push_back(Point(R.x,R.y+R.height));
	src_pts.push_back(Point(R.x+R.width,R.y+R.height));

	dst_pts.push_back(Point2f(center.x-radius,center.y-radius));
	dst_pts.push_back(Point2f(center.x+radius,center.y-radius));
	dst_pts.push_back(Point2f(center.x-radius,center.y+radius));
	dst_pts.push_back(Point2f(center.x+radius,center.y+radius));

	Mat transmtx = getPerspectiveTransform(src_pts,dst_pts);

	//int delX = 2*radius - R.width;
	//int delY = 2*radius - R.height;

	Mat transformed = Mat::zeros(image.rows, image.cols, CV_8UC3);
	warpPerspective(image, transformed, transmtx, Size(image.cols, image.rows));
	
	if(debugMode){	
		showReducedImage("transformed", transformed, 0.2);
		//imshow("transformed", transformed);

		imwrite("circle.jpg", cntimg);
		showReducedImage("circle", cntimg, 0.2);
		waitKey(0);
	}

	//transform points
	//perspectiveTransform(paddedCorners, transformedCorners, transmtx);
	imwrite("flatten.jpg", transformed);


	return true;

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
		imwrite("color.jpg", colorMat);
		waitKey(0);
	}

}


bool findBoundary(Mat img, Mat colorMat, vector<Point> &boundary)
{
	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours( colorMat.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
	//find biggest contour
	Mat cntimg = Mat::zeros(img.rows, img.cols, CV_8UC1);
	
	double maxArea = 0;
	int indMaxArea = -1;
	for(int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);

		if(isContourConvex(contours[i]))
			continue;

		if(area > 0.8*img.rows*img.cols)
			continue;

		if( area > maxArea)
		{
			maxArea = area;
			indMaxArea = i;
		}
	}
	
	if(indMaxArea < 0)
	{
		cout << "Color boundary contour not found ... quitting" << endl;
		return false;
	}

	if(debugMode)	
        	cout << "found color boundary.. " << endl;

	//largest contour
	drawContours(cntimg, contours, indMaxArea, Scalar(255));
	
	if(debugMode){	
		showReducedImage("largest contour", cntimg, 0.2);
		imwrite("boundary.jpg", cntimg);
		waitKey(0);
	}

		
	//vector<Point> approxcnt;
	approxPolyDP(Mat(contours[indMaxArea]), boundary, 2, true);


	//draw points for largest contour
	for(int i = 0; i < boundary.size(); i++){
		circle(img, boundary[i], 4, Scalar(255, 0, 0), -1);
		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(img, txt, boundary[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255 , 255));
	}

	if(debugMode){
		showReducedImage("boundary Points", img, 0.2);
		waitKey(0);	
	}

	return true;
}



int main( int argc, char** argv)
{
	//read image	
	Mat input = imread(argv[1]);
	

	//debugMode
	if(argc > 2){
		if(strcmp(argv[2],"1")==0)
			debugMode = true;
	}

	if(argc > 3){
		if(strcmp(argv[3],"1")==0)
			isBoundary = true;
	}
	
	if(argc > 4){
		if(strcmp(argv[4],"1")==0)
			isResize = true;
	}


	Mat img;
	input.copyTo(img);

	Mat imgCpy;
	img.copyTo(imgCpy);
	
	//threshold by red color
	Mat red;
	vector<Point> approxcnt;
	vector<Point> corners;
	vector<int> cornersInd;	
	if(isBoundary)
	{
		//extract color of boundary	
		extractColor(img, red);	
		
		//find boundary
		findBoundary(img, red, approxcnt);

		//find corners by boundary
		findCornersByBoundary(approxcnt, corners, cornersInd, 2.5);
	}

	//find circular blobs
	else		
		if(!findCornersBySquares(img, corners))
			return -1;
	

	if(corners.size() != 4)
	{
		cout << "4 corners are not found ... quitting" << endl;
		return 0;
	}	

	//sort corner points in clockwise order starting from bottom-right point
	vector<Point2f> sortedCorners(4);
	vector<int> sortedCornersInd(4);
	Point2f avgCorners;
	avgCorners.x = ((corners[0].x + corners[1].x + corners[2].x + corners[3].x)/4);
	avgCorners.y = ((corners[0].y + corners[1].y + corners[2].y + corners[3].y)/4);

	for(int i = 0; i < corners.size(); i++)
	{	
		approxcnt.push_back(corners[i]);
		if(corners[i].x < avgCorners.x && corners[i].y < avgCorners.y)
			sortedCorners[2] = corners[i];

		if(corners[i].x > avgCorners.x && corners[i].y < avgCorners.y)
			sortedCorners[3] = corners[i];		

		if(corners[i].x > avgCorners.x && corners[i].y > avgCorners.y)
			sortedCorners[0] = corners[i];
			
		if(corners[i].x < avgCorners.x && corners[i].y > avgCorners.y)
			sortedCorners[1] = corners[i];			
	}
	
	//print indices of corners to show the sort order
	for(int i = 0; i < sortedCorners.size(); i++)
	{
		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(imgCpy, txt, sortedCorners[i], CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 0 , 255)); 
	}


	//bounding box	
	Rect boundRect = boundingRect(approxcnt);

	// rotated rectangle
	RotatedRect rotBoundRect = minAreaRect(approxcnt);
    	Point2f rect_points[4]; rotBoundRect.points( rect_points );   //clock-wise order starting with bottom left corner
	
	//draw bounding box and rotated rectangle
	rectangle(img, boundRect, Scalar(255, 0, 0));

	for( int j = 0; j < 4; j++ )
	{
		line( img, rect_points[j], rect_points[(j+1)%4], Scalar(0, 255, 0), 1, 8 );
		String txt;
		stringstream ss;
		ss << (j+1);
		txt = ss.str();
		putText(img, txt, rect_points[j], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0 , 255));
	}
		

	if(debugMode){
        	showReducedImage("image", img, 0.2);
	        imwrite("boundingBoxes.jpg", img);
	        waitKey(0);
	}

	vector<Point2f> src(4), dst(4);
	
	src[0] = rect_points[1]; src[1] = rect_points[2]; src[2] = rect_points[3]; src[3] = rect_points[0];

	dst[0] = Point2f(0, 0);  //2
	dst[1] = Point2f(boundRect.width, 0); //3
	dst[2] = Point2f(boundRect.width, boundRect.height);  //0
	dst[3] = Point2f(0, boundRect.height); //1
		
	Mat ppt = getPerspectiveTransform(src, dst);
	
	Mat dstImg;	
	warpPerspective(imgCpy, dstImg, ppt, Size(boundRect.width, boundRect.height));

	//rotated boundingbox is mapped to non-rotated bounding box
	if(debugMode){
		showReducedImage("transformed1", dstImg, 0.2);
		imwrite("transformed1.jpg", dstImg);
		waitKey(0);
	}	

	//corners after prespective transform
	vector<Point2f> warpCorners, warpcntMaxArea;

	perspectiveTransform(sortedCorners, warpCorners, ppt);

	vector<Point2f> cntMaxArea;
	
	src[0] = warpCorners[2]; src[1] = warpCorners[3]; src[2] = warpCorners[0]; src[3] = warpCorners[1];

	Mat ppt2 = getPerspectiveTransform(src, dst);

	Mat dstImg2;
	warpPerspective(dstImg, dstImg2, ppt2, Size(boundRect.width, boundRect.height));
	
	//corners are mapped to non-rotated bounding box
	if(debugMode){
		showReducedImage("transformed2", dstImg2, 0.2);
		imwrite("transformed2.jpg", dstImg2);
		waitKey(0);
	}


	if(!detectCircle(dstImg2.clone()))
		return -1;

	return 1;
}


/*bool readConfigFile()
{
	string line;
	ifstream config (configFile);
  	if (config.is_open())
  	{
    		while ( getline (config,line) )
    		{
      			size_t delimPos = line.find_first_of(";");
			string param = line.substr(0, delimPos+1);
			string value = line.substr(delimPos+1, line.size());
			
			//read debug mode value
			if(strcmp(param, "debugMode")==0){
				if(strcmp(value,"1")==0)
				   debugMode = true;	

			}
			
    		}
	    
	   config.close();

       }

       else return 0;		
	
	
      return true;

}
*/





