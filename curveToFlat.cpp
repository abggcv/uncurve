#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
 
using namespace cv;
using namespace std;


static bool checkPointInsideRect(Point p, Rect r)
{
	if(p.x >= r.x && p.y >= r.y && p.x < r.x + r.width && p.y < r.y + r.height)
		return true;

	return true;
}

//f = y1+m(x-x1)-y

static bool checkPoint(Point p, double m, Point p1, double chkVal)
{
	double f = p1.y + (m*(p.x-p1.x)) - p.y;

	if(chkVal*f > 0)
		return false;

	return true;
}


static void applyWarping(Mat img, Mat &out, vector<Point> points, int w)  //order of points (tl, bl, tr, br)
{
	Mat temp;
	Rect bndRect = boundingRect(points);

	img(bndRect).copyTo(temp);

	vector<Point2f> src;
	for(int i = 0; i < points.size(); i++)
		src.push_back(Point2f(points[i].x, points[i].y));

	double l1 = sqrt((points[0].x - points[1].x)*(points[0].x - points[1].x) + (points[0].y - points[1].y)*(points[0].y - points[1].y));
	double l2 = sqrt((points[2].x - points[3].x)*(points[2].x - points[3].x) + (points[2].y - points[3].y)*(points[2].y - points[3].y));

	double l = (l1 > l2)?l1:l2;

	vector<Point2f> dst;
	dst.push_back(Point2f(0, 0));
	dst.push_back(Point2f(0, l));
	dst.push_back(Point2f(w, 0));
	dst.push_back(Point2f(w, l));

	Mat pt = getPerspectiveTransform(src, dst);

	//Mat out;
	//perspectiveTransform(img, out, pt);

	warpPerspective(img, out, pt, Size(w, (int)l));

	//imshow("test", out);
	//waitKey(0);

}

void findCorners(vector<Point2f> contour, vector<int> &corners, vector<Point2f> &ret, int max_corners, int min_angle, int min_distance)
{
	double prevAngle = 0; //in degrees
	double distance = 0;

	ret.reserve(max_corners);

	corners.reserve(max_corners);

	int cc = 0;	

	for(int i = 0; i < contour.size() + 3; i++)
	{
		Point2f p1, p2, p3;

		if(i > 2)
		{			
			if(i - contour.size() < 0)
			{
				p1 = contour[i-2];		
				p2 = contour[i-1];
				p3 = contour[i];
			}
		
			else if(i - contour.size() == 0)
			{
				p1 = contour[i-2];
				p2 = contour[i-1];
				p3 = contour[0];
			}
			else if(i - contour.size() == 1)
			{
				p1 = contour[i-2];
				p2 = contour[0];
				p3 = contour[1];
			}
			else if(i - contour.size() == 2)
			{
				p1 = contour[i - contour.size() - 2];
				p2 = contour[i - contour.size() - 1];
				p3 = contour[i - contour.size()];
			}

			 //Calculate angle between points 1 and 3
			 double currAngle = atan2( p1.y - p3.y, p1.x - p3.x ) * 180 / CV_PI;
			 if( currAngle < 0 )                
				 currAngle = (currAngle * -1);

			 if( i > 3 )
			 {
				 //calculate the difference between this angle and the previous one            
				 double diffAngle = prevAngle - currAngle;
 				
				 if( diffAngle < 0 ) //Make sure sign is positive                
					 diffAngle = (diffAngle * (-1));
                 
				 //Add point to return array if angle diff is above threshold
				 if( diffAngle > min_angle )                
				 {                    
					 //Ignore points that are closer than "min_distance pixels" to the previous point                    
					 if( cc > 0 )                    
					 {                        
						 double dx = p1.x - ret[cc - 1].x;                 
						 double dy = p1.y - ret[cc - 1].y;

						 distance = sqrtf( (dx * dx) + (dy * dy) );
                         
						 if( distance >= min_distance )                        
						 {                            
							 ret[cc] = p1;							
							 corners[cc] = i-2;                            
							 cc++;      
							 cout << "corner found: " << i-2 << endl;

						 }
					 }
					 
					 else                    					 
					 {                        						 
						 ret[cc] = p1;
						 corners[cc] = i-2;
						 cc++;
						 cout << "corner found: " << i-2 << endl;
					 }
                     					 
					 if( cc > max_corners-1 )                        						 					 
						 break;                
				 }
            
			 }
             
			 prevAngle = currAngle;		
		}
	}	//end of for loop

	if( cc < max_corners )
		cout << "corners found : " << cc << " of " << max_corners << " corners" << endl;

} //end of findCorners



void findCorners2(vector<Point2f> cnt, vector<Point2f> &cornerPts, vector<int> &cornerInd, double threshAngle)
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

	cout << "corners found: " << endl;

	for(int i = 0; i < cornerInd.size(); i++)
		cout << cornerInd[i] << ", ";

	cout << endl;

}


void showReducedImage(string title, Mat img)
{
	Mat temp;
	resize(img, temp, Size(), 0.5, 0.5, CV_INTER_AREA);

	imshow(title, temp);
}



void detectCircle(Mat img)
{
	//apply padding border around image
	Mat image = Mat::zeros((int)(1.6*img.rows), (int)(1.6*img.cols), CV_8UC3);

	Rect imgRect = Rect(0, (int)(0.3*img.rows), img.cols, img.rows);

	img.copyTo(image(imgRect));
	showReducedImage("padded", image);
	//imshow("padded", image);
	waitKey(0);

	Mat hsv;
	cvtColor(image.clone(), hsv, CV_BGR2HSV);

	//Mat thresh;
	//threshold(gray.clone(), thresh, 30, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	Mat thresh;
	inRange(hsv, Scalar(0, 0, 0, 0), Scalar(180, 255, 80, 0), thresh);  //detect black color

	showReducedImage("threshold", thresh);
	//imshow("threshold", thresh);
	waitKey(0);

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours( thresh.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//draw contours
	Mat cntimg = Mat::zeros(image.rows, image.cols, CV_8UC3);

	int ind = 0;

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

	//vector<Point> approxcnt;

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

	int delX = 2*radius - R.width;
	int delY = 2*radius - R.height;

	Mat transformed = Mat::zeros(image.rows, image.cols + delX, CV_8UC3);
	warpPerspective(image, transformed, transmtx, Size(image.cols+delX, image.rows));
	
	showReducedImage("transformed", transformed);
	//imshow("transformed", transformed);

	//imshow("circle", cntimg);
	showReducedImage("circle", cntimg);
	waitKey(0);

}


int main( int argc, char** argv)
{
	//read image
	//Mat input = imread("C:\\Users\\abgg\\Downloads\\od\\new\\curved\\20160407_215011.jpg");
	Mat input = imread(argv[1]);

	//resize image
	Mat img;
	//resize(input, img, Size(), 0.2, 0.2);
	input.copyTo(img);

	Mat imgCpy;
	img.copyTo(imgCpy);

	Mat img2;
	img.copyTo(img2);

	//threshold by red color
	
	//convert to hsv
	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);

	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);

	Mat1b mask1, mask2;
    inRange(hsv, Scalar(0, 70, 100), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(170, 70, 100), Scalar(180, 255, 255), mask2);

    Mat1b red = mask1 | mask2;

	//morphological operations
	dilate(red, red, Mat());
	erode(red, red, Mat());
	morphologyEx(red,red,MORPH_CLOSE,getStructuringElement( MORPH_ELLIPSE,Size(7,7)));
	
	showReducedImage("red", red);
	//imshow("red", red);
	waitKey(0);

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours( red.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	cout << "found contours .. " << endl;

	//draw contours
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
		cout << "index of max. area contour not found ... quitting" << endl;
		return 0;
	}

	cout << "found largest contour.. " << endl;

	//largest contour
	drawContours(cntimg, contours, indMaxArea, Scalar(255));
	showReducedImage("largest contour", cntimg);
	//imshow("largest contour", cntimg);
	waitKey(0);

	vector<Point2f> approxcnt;
	approxPolyDP(Mat(contours[indMaxArea]), approxcnt, 2, true);

	//find corners
	vector<Point2f> corners;
	vector<int> cornersInd;

	findCorners2(approxcnt, corners, cornersInd, 2.5);

	if(corners.size() != 4)
	{
		cout << "4 corners are not found ... quitting" << endl;
		return 0;
	}

	//draw points for largest contour
	for(int i = 0; i < approxcnt.size(); i++){
		circle(img, approxcnt[i], 4, Scalar(255, 0, 0), -1);
		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(img, txt, approxcnt[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255 , 255));
	}

	//bouding box	
	Rect boundRect = boundingRect(contours[indMaxArea]);

	Rect bb2 = Rect(boundRect.x-20, boundRect.y-20, boundRect.width+60, boundRect.height+60); //extend bounding box to cover more area like 20 pixels on all sides

	// rotated rectangle
	RotatedRect rotBoundRect = minAreaRect(contours[indMaxArea]);		
    Point2f rect_points[4]; rotBoundRect.points( rect_points );

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
	showReducedImage("image", img);
	//imshow("image", img);
	waitKey(0);


	//apply perspective correction to circle inside bounding box
	Mat bbImg;
	img(bb2).copyTo(bbImg);


	vector<Scalar> colorVec(4);
	colorVec[0] = Scalar(0, 255, 0);
	colorVec[1] = Scalar(50, 255, 112); 
	colorVec[2] = Scalar(120, 255, 20);
	colorVec[3] = Scalar(10, 150, 0);

	//sort corner points in clockwise order starting from bottom-right point
	vector<Point2f> sortedCorners(4);
	vector<int> sortedCornersInd(4);
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

	//print indices of corners to show the sort order
	for(int i = 0; i < sortedCorners.size(); i++)
	{
		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(imgCpy, txt, sortedCorners[i], CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 0 , 255)); 
	}

	
	Point2f boundRectPoints[4]; 

	boundRectPoints[0] = Point2f((float)(boundRect.x + boundRect.width), (float)(boundRect.y + boundRect.height));
	boundRectPoints[1] = Point2f((float)boundRect.x, (float)(boundRect.y + boundRect.height));
	boundRectPoints[2] = Point2f((float)boundRect.x, (float)boundRect.y);
	boundRectPoints[3] = Point2f((float)(boundRect.x + boundRect.width), (float)boundRect.y);

	//extract each curved side separately
	vector<vector<Point>> curvedSides(2);

	vector<Point> largestcnt = contours[indMaxArea];

	//bounds for left and right curved sides

	//y - y1 = (y2-y1)*(x-x1)/(x2-x1)
	//f = y1+m(x-x1)-y
	
	double m1, m2;
	m1 = (sortedCorners[2].y - sortedCorners[1].y)/(sortedCorners[2].x - sortedCorners[1].x);
	m2 = (sortedCorners[3].y - sortedCorners[0].y)/(sortedCorners[3].x - sortedCorners[0].x);

	cout << "m1: " << m1 << " and m2: " << m2 << endl;

	double f1, f2;
	f1 = sortedCorners[1].y + m1*(avgCorners.x - sortedCorners[1].x) - avgCorners.y;
	f2 = sortedCorners[0].y + m2*(avgCorners.x - sortedCorners[0].x) - avgCorners.y;

	cout << "f1: " << f1 << " and f2: " << f2 << endl;

	vector<Point> leftSide, rightSide;

	for(int i = 0; i < largestcnt.size(); i++)
	{
		if(checkPoint(largestcnt[i], m1, corners[1], f1))
			leftSide.push_back(largestcnt[i]);

		else
			if(checkPoint(largestcnt[i], m2, sortedCorners[0], f2))
				rightSide.push_back(largestcnt[i]);
	}

	cout << "size of image: " << img.size() << endl;

	cout << "left side: " << endl;
	cout << leftSide << endl;

	cout << endl;

	cout << "right side: " << endl;
	cout << rightSide << endl;
	cout << endl;

	curvedSides.push_back(leftSide);
	curvedSides.push_back(rightSide);

	//draw curvedsides
	for(int j = 0; j < leftSide.size(); j++)			
		circle(imgCpy, leftSide[j], 3, Scalar(0, 255, 0), -1);

	for(int j = 0; j < rightSide.size(); j++)			
		circle(imgCpy, rightSide[j], 3, Scalar(255, 0, 0), -1);

	cout << "length of left curved side: " << arcLength(leftSide, false) << endl;

	cout << "verticle height of left curved side: " << abs(sortedCorners[2].y - sortedCorners[1].y) << endl;

	cout << "verticle height of right curved side: " << abs(sortedCorners[3].y - sortedCorners[0].y) << endl;

	cout << "length of right curved side: " << arcLength(rightSide, false) << endl;

	/*for(int i = 0; i < 2; i++)
		for(int j = 0; j < curvedSides[i].size(); j++)
			circle(imgCpy, curvedSides[i][j], 3, colorVec[i], -1);
			*/
	vector<Point2f> src(4), dst(4);

	//src[0] = boundRectPoints[2]; src[1] = boundRectPoints[3]; src[2] = boundRectPoints[0]; src[3] = boundRectPoints[1]; 
	
	//src[0] = rect_points[2]; src[1] = rect_points[3]; src[2] = rect_points[0]; src[3] = rect_points[1];

	//src[0] = rect_points[2]; src[1] = rect_points[3]; src[2] = boundRectPoints[0]; src[3] = boundRectPoints[1]; 

	//src[0] = boundRectPoints[2]; src[1] = boundRectPoints[3]; src[2] = rect_points[0]; src[3] = rect_points[1];
	src[0] = sortedCorners[2]; src[1] = sortedCorners[3]; src[2] = sortedCorners[0]; src[3] = sortedCorners[1];
	
	//dst[0] = Point2f(boundRect.x, boundRect.y); dst[1] = Point2f(boundRect.x + boundRect.width, boundRect.y);
	//dst[2] = Point2f(boundRect.x + boundRect.width, boundRect.y + boundRect.height);

	int h1 = src[3].y - src[0].y;
	int h2 = src[2].y - src[1].y;

	cout << "h1: " << h1 << " and h2: " << h2 << endl;

	int h = (h1 > h2)?h1:h2;

	cout << "h: " << h << " and boundRect height: " << boundRect.height << endl;

	dst[0] = Point2f(0, 0);
	dst[1] = Point2f(boundRect.width, 0);
	dst[2] = Point2f(boundRect.width, boundRect.height);
	dst[3] = Point2f(0, boundRect.height);
	
	/*for( int j = 0; j < 4; j++ )
	{
		String txt;
		stringstream ss;
		ss << (j+1);
		txt = ss.str();
		putText(img, txt, rect_points[j], CV_FONT_HERSHEY_PLAIN, 2, Scalar(255, 0 ,0)); 
		line( img, rect_points[j], rect_points[(j+1)%4], Scalar(255, 0, 255), 1, 8 );

		putText(img, txt, boundRectPoints[j], CV_FONT_HERSHEY_PLAIN, 2, Scalar(0, 255 ,0)); 

	}
	*/
	Mat ppt = getPerspectiveTransform(src, dst);

	cout << "applying perspective transform" << endl;
	
	Mat dstImg;	
	warpPerspective(imgCpy, dstImg, ppt, Size(boundRect.width, h));

	showReducedImage("transformed1", dstImg);
	//imshow("transformed1", dstImg);
	waitKey(0);

	//corners after prespective transform
	vector<Point2f> warpCorners, warpcntMaxArea;

	perspectiveTransform(sortedCorners, warpCorners, ppt);

	vector<Point2f> cntMaxArea;
	
	for(int i = 0; i < contours[indMaxArea].size(); i++)
		cntMaxArea.push_back(Point2f(contours[indMaxArea][i].x, contours[indMaxArea][i].y));

	perspectiveTransform(cntMaxArea, warpcntMaxArea, ppt);


	//bounding box for warped contours

	//Rect wbb = boundingRect(warpcntMaxAreaMaxArea);

	detectCircle(dstImg.clone());

	int minX = 10000;
	int maxX = 0;

	int indMin = -1;
	int indMax = -1;

	for(int i = 0; i < warpcntMaxArea.size(); i++)
	{
		if(minX > warpcntMaxArea[i].x)
		{
			minX = warpcntMaxArea[i].x;
			indMin = i;
		}

		if(maxX < warpcntMaxArea[i].x)
		{
			maxX = warpcntMaxArea[i].x;
			indMax = i;
		}

		String txt;
		stringstream ss;
		ss << (i+1);
		txt = ss.str();
		putText(dstImg, txt, warpcntMaxArea[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255 , 255));
		circle(dstImg, warpcntMaxArea[i], 3, Scalar(255, 0, 0), -1);

	}	//circle(dstImg, warpCorners[i], 3, Scalar(255, 0, 0), -1);
	

	//line(dstImg, Point(0, (int)(h/2)), Point(boundRect.width, (int)(h/2)), Scalar(0, 0, 255), 2);

	line(dstImg, warpcntMaxArea[0], warpcntMaxArea[9], Scalar(0, 0, 255), 2);
	line(dstImg, warpcntMaxArea[1], warpcntMaxArea[8], Scalar(0, 0, 255), 2);
	line(dstImg, warpcntMaxArea[2], warpcntMaxArea[7], Scalar(0, 0, 255), 2);
	line(dstImg, warpcntMaxArea[3], warpcntMaxArea[6], Scalar(0, 0, 255), 2);

	line(dstImg, warpcntMaxArea[4], warpcntMaxArea[5], Scalar(255, 0, 255), 2);
	line(dstImg, warpcntMaxArea[11], warpcntMaxArea[10], Scalar(255, 0, 255), 2);

	Mat dst1, dst2;

	//h = 395;

	h1 = (int)(h/2);
	int w = dstImg.cols;

	h = 395;

	h2 = (int)(h/2);

	Rect r1 = Rect(0, 0, w, h1);
	Rect r2 = Rect(0, h1, w, h1);

	dstImg(r1).copyTo(dst1);
	dstImg(r2).copyTo(dst2);

	//imshow("dst2", dst2);

	vector<Point2f> src1(4), src2(4);
	vector<Point2f> dstPts1(4), dstPts2(4);

	src1[0] = Point2f(warpCorners[2].x-minX, warpCorners[2].y); src1[1] = Point2f(warpCorners[3].x-minX, warpCorners[3].y); 
	src1[2] = Point2f(w-minX, h1); src1[3] = Point2f(0, h1);
	
	dstPts1[0] = Point2f(0, 0); dstPts1[1] = Point2f(w, 0); dstPts1[2] = Point2f(w, h2); dstPts1[3] = Point2f(0, h2);
	
	src2[0] = Point2f(0, 0); src2[1] = Point2f(w-minX, 0); src2[2] = Point2f(warpCorners[0].x-minX, warpCorners[0].y - h1); 
	src2[3] = Point2f(warpCorners[1].x-minX, warpCorners[1].y - h1);
	dstPts2[0] = Point2f(0, 0); dstPts2[1] = Point2f(w, 0); dstPts2[2] = Point2f(w, h-h2); dstPts2[3] = Point2f(0, h-h2); 

	Mat ppt1 = getPerspectiveTransform(src1, dstPts1);
	Mat ppt2 = getPerspectiveTransform(src2, dstPts2);

	Mat dstImg21(h2, w, CV_8UC3), dstImg22(h-h2, w, CV_8UC3);

	warpPerspective(dst1, dstImg21, ppt1, dstImg21.size());
	warpPerspective(dst2, dstImg22, ppt2, dstImg22.size());

	cout << "perspective transformation done! " << endl;

	//imshow("dstImg22", dstImg22);

	Mat dstImg2(h, w, CV_8UC3);

	r1 = Rect(0, 0, w, h2);
	r2 = Rect(0, h2, w, h - h2);

	dstImg21.copyTo(dstImg2(r1));
	dstImg22.copyTo(dstImg2(r2));

	//show output
	//showReducedImage("red", red);
	//imshow("red", red);
	//imshow("input copy", imgCpy);
	//imshow("input", img);
	showReducedImage("contours", cntimg);
	//imshow("contours", cntimg);
	//imshow("dst", dstImg);
	//imshow("final", dstImg2);
	waitKey(0);

	return 1;
}


//****************************\\
	
	/*Mat dst2Img;
	dstImg.copyTo(dst2Img);

	//bouding box	
	Rect boundRect2 = boundingRect(warpcntMaxArea);

	Mat flatImg2;

	vector<Mat> outImages2;

	int finalHt2 = 0;

	int maxWidth2 = 0;

	//Rect rect2;

	for(int i = 0; i < 5; i++)
	{
		vector<Point> p(4);
		
		//for(int j = 0; j < 5; j++)
			//p.push_back(approxcnt[ind[i][j]]);

		p[0] = warpcntMaxArea[ind[i][0]];
		p[1] = warpcntMaxArea[ind[i][1]];
		p[2] = warpcntMaxArea[ind[i][2]];
		p[3] = warpcntMaxArea[ind[i][3]];

		Mat out;

		applyWarping(dst2Img.clone(), out, p, boundRect2.width);

		cout << "width of output - " << i << " is: " << out.cols << endl;
		cout << "height of output - " << i << " is: " << out.rows << endl;

		outImages2.push_back(out);

		finalHt2 += out.rows;

		maxWidth2 = (maxWidth2 >= out.cols)?maxWidth2:out.cols;

	}

	vconcat(outImages2, flatImg2);

	imshow("flat final", flatImg2);
	waitKey(0);
	*/


	//*********************************//




//detectCircle(bbImg);

	//unwarping each part
	/*vector<vector<int>> ind(5);

	ind[0].push_back(11); ind[0].push_back(0); ind[0].push_back(10); ind[0].push_back(9);
	ind[1].push_back(0); ind[1].push_back(1); ind[1].push_back(9); ind[1].push_back(8);
	ind[2].push_back(1); ind[2].push_back(2); ind[2].push_back(8); ind[2].push_back(7);
	ind[3].push_back(2); ind[3].push_back(3); ind[3].push_back(7); ind[3].push_back(6);
	ind[4].push_back(3); ind[4].push_back(4); ind[4].push_back(6); ind[4].push_back(5);


	//cout << "ind[0][0]: " << ind[0][0] << endl << endl;

	vector<Point> p(4);
	p[0] = approxcnt[11];
	p[1] = approxcnt[0];
	p[2] = approxcnt[10];
	p[3] = approxcnt[9];
	
	Mat out;

	applyWarping(img2.clone(), out, p); 
	

	Mat flatImg;

	vector<Mat> outImages;

	int finalHt = 0;

	int maxWidth = 0;

	Rect r;

	for(int i = 0; i < 5; i++)
	{
		vector<Point> p(4);
		
		//for(int j = 0; j < 5; j++)
			//p.push_back(approxcnt[ind[i][j]]);

		p[0] = approxcnt[ind[i][0]];
		p[1] = approxcnt[ind[i][1]];
		p[2] = approxcnt[ind[i][2]];
		p[3] = approxcnt[ind[i][3]];

		Mat out;

		applyWarping(img2.clone(), out, p, boundRect.width);

		//cout << "width of output - " << i << " is: " << out.cols << endl;
		//cout << "height of output - " << i << " is: " << out.rows << endl;

		outImages.push_back(out);

		finalHt += out.rows;

		maxWidth = (maxWidth >= out.cols)?maxWidth:out.cols;

	}


	//Mat vcatImg = Mat::zeros(finalHt, maxWidth, CV_8UC3);

	//for(int i = 0; i < outImages.size(); i++)

	vconcat(outImages, flatImg);
	*/
	//imshow("flat final", flatImg);
	//waitKey(0);





/*

//Retangles bounding curved sides
	Rect rL, rR;

	//Left Rectangle
	rL.x = boundRect.x;
	rL.y = sortedCorners[2].y;
	rL.width = (sortedCorners[2].x > sortedCorners[1].x)?sortedCorners[2].x:sortedCorners[1].x - boundRect.x;
	rL.height = sortedCorners[1].y - sortedCorners[2].y;

	rectangle(imgCpy, rL, Scalar(255, 0, 0));

	//right Rectangle
	rR.x = sortedCorners[3].x;
	rR.y = sortedCorners[3].y;
	rR.width = boundRect.x + boundRect.width - (sortedCorners[3].x < sortedCorners[0].x)?sortedCorners[3].x:sortedCorners[0].x;
	rR.height = sortedCorners[3].y - sortedCorners[0].y;

	rectangle(imgCpy, rR, Scalar(0, 255, 0));

//extract each curved side separately
	vector<vector<Point>> curvedSides(2);

	vector<Point> largestcnt = contours[indMaxArea];

	for(int i = 0; i < largestcnt.size(); i++)
	{



	}


	//draw curved sides
	//for(int i = 0; i < curvedSides.size(); i++)
	drawContours(imgCpy, curvedSides, 0, Scalar(0, 255, 0));
	drawContours(imgCpy, curvedSides, 1, Scalar(50, 255, 112));
	drawContours(imgCpy, curvedSides, 2, Scalar(120, 255, 20));
	drawContours(imgCpy, curvedSides, 3, Scalar(10, 150, 0));

	*/