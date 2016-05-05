#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
 
using namespace cv;
using namespace std;


void fillHoles(Mat &bw, Mat &out)
{
	// Threshold.
    // Set values equal to or above 220 to 0.
    // Set values below 220 to 255.
    Mat im_th;
    threshold(bw.clone(), im_th, 220, 255, THRESH_BINARY_INV);
     
    // Floodfill from point (0, 0)
    Mat im_floodfill = im_th.clone();
    floodFill(im_floodfill, cv::Point(0,0), Scalar(255));
     
    // Invert floodfilled image
    Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);
     
    // Combine the two images to get the foreground.
    out = (im_th | im_floodfill_inv);
}


static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


int main( int argc, char** argv)
{
	//read image
	Mat input = imread("C:\\Users\\abgg\\Downloads\\od\\new\\curved\\20160407_215011.jpg");

	//resize image
	Mat img;
	resize(input, img, Size(), 0.2, 0.2);
	//input.copyTo(img);

	Mat imgCpy;
	img.copyTo(imgCpy);

	//threshold by red color
	
	//convert to hsv
	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);

	//detect red color by range
	Mat red;
	inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), red);

	//morphological operations
	dilate(red, red, Mat());
	erode(red, red, Mat());
	morphologyEx(red,red,MORPH_CLOSE,getStructuringElement( MORPH_ELLIPSE,Size(17,17)));

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours( red.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//draw contours
	Mat cntimg = Mat::zeros(img.rows, img.cols, CV_8UC1);
	
	double maxArea = 0;
	int indMaxArea = -1;
	for(int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);

		if(area > 0.8*img.rows*img.cols)
			continue;

		if( area > maxArea)
		{
			maxArea = area;
			indMaxArea = i;
		}
	}
	
	vector<Point2f> approxcnt;
	approxPolyDP(Mat(contours[indMaxArea]), approxcnt, 4, true);

	//approxPolyDP(Mat(contours[indMaxArea]), approxcnt, arcLength(Mat(contours[indMaxArea]), true)*0.02, true);

	//cout << "drawing approx contours" << endl;

	//drawContours(img, approxcnt, 0, Scalar(255, 0, 0));

	//find corners
	vector<Point2f> corners;

	for(int i = 0; i < approxcnt.size(); i++){
		circle(img, approxcnt[i], 4, Scalar(255, 0, 0), -1);
		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(img, txt, approxcnt[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255 , 255));
	}

	int delX1, delX2, delY1, delY2;

	for(int i = 0; i < approxcnt.size()-1; i++)
	{
		Point p1, p2, p3;

		p1 = approxcnt[i];
		p2 = approxcnt[i+1];

		if(i == approxcnt.size()-2)
			p3 = approxcnt[0];
		else
			p3 = approxcnt[i+2];

		delX1 = abs(p1.x - p2.x);
		delX2 = abs(p2.x - p3.x);
		delY1 = abs(p1.y - p2.y);
		delY2 = abs(p3.y - p2.y);

		if(((delX1 < delY1) && (delY2 < delX2)) || ((delX1 > delY1) && (delY2 > delX2)))
		{
			corners.push_back(Point2f(p2.x, p2.y));
			circle(img, p2, 4, Scalar(0, 255, 0), -1);
		}
	}

	if(corners.size() != 4)
	{
		cout << "4 corners are not found ... quitting" << endl;
		return 0;
	}

		
	if(indMaxArea < 0)
		return 0;
	
	//sort corner points in clockwise order starting from bottom-right point
	vector<Point2f> sortedCorners(4);
	Point2f avgCorners;
	avgCorners.x = ((corners[0].x + corners[1].x + corners[2].x + corners[3].x)/4);
	avgCorners.y = ((corners[0].y + corners[1].y + corners[2].y + corners[3].y)/4);

	for(int i = 0; i < corners.size(); i++)
	{
		if(corners[i].x < avgCorners.x && corners[i].y < avgCorners.y)
			sortedCorners[2] = corners[i];

		if(corners[i].x > avgCorners.x && corners[i].y < avgCorners.y)
			sortedCorners[3] = corners[i];

		if(corners[i].x > avgCorners.x && corners[i].y > avgCorners.y)
			sortedCorners[0] = corners[i];

		if(corners[i].x < avgCorners.x && corners[i].y > avgCorners.y)
			sortedCorners[1] = corners[i];
	}


	for(int i = 0; i < sortedCorners.size(); i++)
	{
		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(img, txt, sortedCorners[i], CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 0 , 255)); 
	}

	//boundingbox
	Rect boundRect = boundingRect(approxcnt);
	rectangle(img, boundRect, Scalar(255, 255, 0));
	RotatedRect rotBoundRect = minAreaRect(approxcnt);		
	drawContours(cntimg, contours, indMaxArea, Scalar(255));
	
	// rotated rectangle
    Point2f rect_points[4]; rotBoundRect.points( rect_points );
	Point2f boundRectPoints[4]; 

	boundRectPoints[0] = Point2f((float)(boundRect.x + boundRect.width), (float)(boundRect.y + boundRect.height));
	boundRectPoints[1] = Point2f((float)boundRect.x, (float)(boundRect.y + boundRect.height));
	boundRectPoints[2] = Point2f((float)boundRect.x, (float)boundRect.y);
	boundRectPoints[3] = Point2f((float)(boundRect.x + boundRect.width), (float)boundRect.y);
    
	/*Rect cropRect;

	cropRect.x = rect_points[1].x;
	cropRect.y = rect_points[2].y;

	cropRect.width = ceil(sqrt((double)((rect_points[2].x - rect_points[3].x)*(rect_points[2].x - rect_points[3].x) + 
					(rect_points[2].y - rect_points[3].y)*(rect_points[2].y - rect_points[3].y))));

	cropRect.height = ceil(sqrt((double)((rect_points[2].x - rect_points[1].x)*(rect_points[2].x - rect_points[1].x) + 
					(rect_points[2].y - rect_points[1].y)*(rect_points[2].y - rect_points[1].y))));
					*/

	vector<Point2f> src(4), dst(4);

	src[0] = boundRectPoints[2]; src[1] = boundRectPoints[3]; src[2] = rect_points[0]; src[3] = rect_points[1];
	/*src[0] = Point2f(boundRect.x, corners[2].y);
	src[1] = Point2f(boundRect.x + boundRect.width, corners[3].y);
	src[2] = Point2f(boundRect.x + boundRect.width, corners[0].y);
	src[3] = Point2f(boundRect.x, corners[1].y);
	*/
	int h1 = src[3].y - src[0].y;
	int h2 = src[2].y - src[1].y;

	cout << "h1: " << h1 << " and h2: " << h2 << endl;

	int h = (h1 > h2)?h1:h2;

	cout << "h: " << h << " and boundRect height: " << boundRect.height << endl;

	//h = boundRect.height-20;

	//sortedCorners
	//src[0] = sortedCorners[2]; src[1] = sortedCorners[3]; src[2] = sortedCorners[0]; src[3] = sortedCorners[1];

	dst[0] = Point2f(0, 0);
	dst[1] = Point2f(boundRect.width, 0);
	dst[2] = Point2f(boundRect.width, h);
	dst[3] = Point2f(0, h);

	for( int j = 0; j < 4; j++ )
	{
		String txt;
		stringstream ss;
		ss << (j+1);
		txt = ss.str();
		putText(img, txt, rect_points[j], CV_FONT_HERSHEY_PLAIN, 2, Scalar(255, 0 ,0)); 
		line( img, rect_points[j], rect_points[(j+1)%4], Scalar(255, 0, 255), 1, 8 );

		putText(img, txt, boundRectPoints[j], CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 255 ,0)); 

	}

	//perspective transform to rotated rect
	/*vector<Point2f> src, dst;
	for(int i = 0; i < sortedCorners.size();i++)
	{
		src.push_back(Point2f(sortedCorners[i].x, sortedCorners[i].y));
		dst.push_back(Point2f(rect_points[i].x, rect_points[i].y));
	}*/

	Mat ppt = getPerspectiveTransform(src, dst);

	
	//perspectiveTransform
	//Mat srcImg;
	//img(cropRect).copyTo(srcImg);

	//imshow("cropped", srcImg);
	
	cout << "applying perspective transform" << endl;
	
	Mat dstImg;
	//logPolar(srcImg,dstImg,Point(srcImg.cols/2,srcImg.rows/2),20,INTER_CUBIC );
	warpPerspective(imgCpy, dstImg, ppt, Size(boundRect.width, h));

	//corners after prespective transform
	vector<Point2f> warpCorners, warpcnt;

	perspectiveTransform(sortedCorners, warpCorners, ppt);

	perspectiveTransform(approxcnt, warpcnt, ppt);

	int minX = 10000;
	int maxX = 0;

	int indMin = -1;
	int indMax = -1;

	for(int i = 0; i < warpcnt.size(); i++)
	{
		if(minX > warpcnt[i].x)
		{
			minX = warpcnt[i].x;
			indMin = i;
		}

		if(maxX < warpcnt[i].x)
		{
			maxX = warpcnt[i].x;
			indMax = i;
		}

		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(dstImg, txt, warpcnt[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255 , 255));
		circle(dstImg, warpcnt[i], 3, Scalar(255, 0, 0), -1);

	}	//circle(dstImg, warpCorners[i], 3, Scalar(255, 0, 0), -1);
	

	//line(dstImg, Point(0, (int)(h/2)), Point(boundRect.width, (int)(h/2)), Scalar(0, 0, 255), 2);

	line(dstImg, warpcnt[0], warpcnt[9], Scalar(0, 0, 255), 2);
	line(dstImg, warpcnt[1], warpcnt[8], Scalar(0, 0, 255), 2);
	line(dstImg, warpcnt[2], warpcnt[7], Scalar(0, 0, 255), 2);
	line(dstImg, warpcnt[3], warpcnt[6], Scalar(0, 0, 255), 2);

	line(dstImg, warpcnt[4], warpcnt[5], Scalar(255, 0, 255), 2);
	line(dstImg, warpcnt[11], warpcnt[10], Scalar(255, 0, 255), 2);

	Mat dst1, dst2;

	h1 = (int)(h/2);
	int w = dstImg.cols;

	Rect r1 = Rect(0, 0, w, h1);
	Rect r2 = Rect(0, h1, w, h1);

	dstImg(r1).copyTo(dst1);
	dstImg(r2).copyTo(dst2);

	imshow("dst2", dst2);

	vector<Point2f> src1(4), src2(4);
	vector<Point2f> dstPts1(4), dstPts2(4);

	src1[0] = Point2f(warpCorners[2].x-minX, warpCorners[2].y); src1[1] = Point2f(warpCorners[3].x-minX, warpCorners[3].y); 
	src1[2] = Point2f(w-minX, h1); src1[3] = Point2f(0, h1);
	
	dstPts1[0] = Point2f(0, 0); dstPts1[1] = Point2f(w, 0); dstPts1[2] = Point2f(w, h1); dstPts1[3] = Point2f(0, h1);
	
	src2[0] = Point2f(0, 0); src2[1] = Point2f(w-minX, 0); src2[2] = Point2f(warpCorners[0].x-minX, warpCorners[0].y - h1); 
	src2[3] = Point2f(warpCorners[1].x-minX, warpCorners[1].y - h1);
	dstPts2[0] = Point2f(0, 0); dstPts2[1] = Point2f(w, 0); dstPts2[2] = Point2f(w, h-h1); dstPts2[3] = Point2f(0, h-h1); 

	Mat ppt1 = getPerspectiveTransform(src1, dstPts1);
	Mat ppt2 = getPerspectiveTransform(src2, dstPts2);

	Mat dstImg21, dstImg22;
	warpPerspective(dst1, dstImg21, ppt1, dst1.size());
	warpPerspective(dst2, dstImg22, ppt2, dst2.size());

	imshow("dstImg22", dstImg22);

	Mat dstImg2(h, w, CV_8UC3);

	dstImg21.copyTo(dstImg2(r1));
	dstImg22.copyTo(dstImg2(r2));

	//show output
	imshow("red", red);
	//imshow("filled red", filledRed);
	imshow("input", img);
	imshow("contours", cntimg);
	imshow("dst", dstImg);
	imshow("final", dstImg2);
	waitKey(0);

	return 1;
}