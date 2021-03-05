#ifndef _renderFace_H_
#define _renderFace_H_
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

using namespace cv;
using namespace std;

#define COLOR Scalar(255, 200,0)

// drawPolyLine draws a poly line by joining 
// successive points between the start and end indices. 
void drawPolyline
(
    Mat& im,
    const vector<Point2f>& landmarks,
    const int start,
    const int end,
    bool isClosed = false
)
{
    // Gather all points between the start and end indices
    vector <Point> points;
    for (int i = start; i <= end; i++)
    {
        points.push_back(cv::Point(landmarks[i].x, landmarks[i].y));
    }
    // Draw polylines. 
    polylines(im, points, isClosed, COLOR, 2, 16);

}


void DrawLandmarks(Mat& im, vector<Point2f>& landmarks)
{
    for (int i = 0; i < landmarks.size(); i++)
    {
        circle(im, landmarks[i], 3, COLOR, FILLED);
    }
}

void DrawFaceRect(Mat& im, vector<Rect>& faces)
{
    for (auto& face : faces)
    {
        line(im, { face.x , face.y }, { face.x , face.y + face.height }, COLOR);
        line(im, { face.x , face.y }, { face.x + face.width , face.y }, COLOR);
        line(im, { face.x + face.width , face.y }, { face.x + face.width , face.y + face.height }, COLOR);
        line(im, { face.x , face.y + face.height }, { face.x + face.width , face.y + face.height }, COLOR);
    }
}


#endif // _renderFace_H_