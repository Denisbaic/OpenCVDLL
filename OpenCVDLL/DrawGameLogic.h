#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

inline void DrawMouseRect(Mat& im, Rect& MouseRect)
{
    line(im, { MouseRect.x , MouseRect.y }, { MouseRect.x , MouseRect.y + MouseRect.height }, Scalar(255, 200, 0));
    line(im, { MouseRect.x , MouseRect.y }, { MouseRect.x + MouseRect.width , MouseRect.y }, Scalar(255, 200, 0));
    line(im, { MouseRect.x + MouseRect.width , MouseRect.y }, { MouseRect.x + MouseRect.width , MouseRect.y + MouseRect.height }, Scalar(255, 200, 0));
    line(im, { MouseRect.x , MouseRect.y + MouseRect.height }, { MouseRect.x + MouseRect.width , MouseRect.y + MouseRect.height }, Scalar(255, 200, 0));
}

inline void DrawMouseDirection(Mat& im, Rect& MouseRect, vector<Point2f>& landmarks)
{
	Point2i const MiddlePoint(MouseRect.x+ MouseRect.width/2, MouseRect.y + MouseRect.height/2);
    line(im, MiddlePoint, landmarks[30], Scalar(255, 200, 0));	
}