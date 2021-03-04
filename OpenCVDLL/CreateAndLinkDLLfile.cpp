#include "pch.h"

#include <cstring>
#include "CreateAndLinkDLLfile.h"
#include "DrawGameLogic.h"

//Exported method that invertes a given boolean.
bool getInvertedBool(bool boolState)
{
    return bool(!boolState);
}

//Exported method that iterates a given int value.
int getIntPlusPlus(int lastInt)
{
    return int(++lastInt);
}

//Exported method that calculates the are of a circle by a given radius.
float getCircleArea(float radius)
{
    return float(3.1416f * (radius * radius));
}

//Exported method that adds a parameter text to an additional text and returns them combined.
char* getCharArray(char* parameterText)
{
    char const* additionalText = " world!";

    if (strlen(parameterText) + strlen(additionalText) + 1 > 256)
    {
        return const_cast<char*>("Error: Maximum size of the char array is 256 chars.");
    }

    char combinedText[256] = "";

    strcpy_s(combinedText, 256, parameterText);
    strcat_s(combinedText, 256, additionalText);

    return static_cast<char*>(combinedText);
}

//Exported method that adds a vector4 to a given vector4 and returns the sum.
float* getVector4(float x, float y, float z, float w)
{
    float* modifiedVector4 = new float[4];

    modifiedVector4[0] = x + 1.0F;
    modifiedVector4[1] = y + 2.0F;
    modifiedVector4[2] = z + 3.0F;
    modifiedVector4[3] = w + 4.0F;

    return (float*)modifiedVector4;
}

void InvertBool()
{
    temp = !temp;
}

bool getBool()
{
    return temp;
}

int init_opencv(int cam_index, char* FaceDetectorCascadeFilePath, char* FaceMarkModelFilePath)
{
    faceDetector.load(FaceDetectorCascadeFilePath);
    if (faceDetector.empty())
        return 1;
    facemark = face::FacemarkLBF::create();
    facemark->loadModel(FaceMarkModelFilePath);
    if (facemark->empty())
        return 2;

    if (cam.open(cam_index) == false)
        return 3;
	
    return 0;
}

void show_frame()
{	
    if (!cam.read(frame))
        return;
	
    // Find face
    vector<Rect> faces;
    // Convert frame to grayscale because
    // faceDetector requires grayscale image.
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Detect faces
    faceDetector.detectMultiScale(gray, faces);

    // Variable for landmarks. 
    // Landmarks for one face is a vector of points
    // There can be more than one face in the image. Hence, we 
    // use a vector of vector of points. 
    vector< vector<Point2f> > landmarks;

    // Run landmark detector
    bool success = facemark->fit(frame, faces, landmarks);

    if (success)
    {
        // If successful, render the landmarks on the face
        for (int i = 0; i < landmarks.size(); i++)
        {
            drawLandmarks(frame, landmarks[i]);
        }

        bool temp = facial_marks.IsMouthOpen;
        facial_marks.IsMouthOpen = facial_marks.IsMouthOpen || IsMouthOpen(landmarks[0]);
        if (temp != facial_marks.IsMouthOpen)
        {
            MouseField.x = landmarks[0][30].x - MouseFieldWidth / 2;
            MouseField.y = landmarks[0][30].y - MouseFieldHeight / 2;
            MouseField.width = MouseFieldWidth;
            MouseField.height = MouseFieldHeight;
            IsSelectedNosePositionForMouseControl = true;
        }
        if (IsSelectedNosePositionForMouseControl)
        {
            DrawMouseRect(frame, MouseField);
            DrawMouseDirection(frame, MouseField, landmarks[0]);
        }

        if (IsEyeOpen(landmarks[0], false))
        {
            drawFaceRect(frame, faces);
        }
    }
    // Display results 
    imshow("Facial Landmark Detection", frame);
    
}

void stop_opencv()
{	
    destroyWindow("Facial Landmark Detection");
    cam.release();
    facemark.release();
    
}

void GetFacialMarks(bool& IsLeftEyeClosed, bool& IsRightEyeClosed, bool& IsMouthOpen)
{
    //IsLeftEyeClosed = facial_marks.IsLeftEyeClosed;
    //IsRightEyeClosed = facial_marks.IsRightEyeClosed;
	
}

/*
int init_dlib(int cam_index, char* predictorFilePath)
{
    if (cam.open(cam_index) == false)
        return 3;

    detector = dlib::get_frontal_face_detector();
    dlib::deserialize(predictorFilePath) >> pose_model;
	
    return 0;
}

void show_frame_dlib()
{
    // Grab a frame
    cv::Mat temp;
    if (!cam.read(temp))
    {
        return;
    }
    // Turn OpenCV's Mat into something dlib can deal with.  Note that this just
    // wraps the Mat object, it doesn't copy anything.  So cimg is only valid as
    // long as temp is valid.  Also don't do anything to temp that would cause it
    // to reallocate the memory which stores the image as that will make cimg
    // contain dangling pointers.  This basically means you shouldn't modify temp
    // while using cimg.
    dlib::cv_image<dlib::bgr_pixel> cimg(temp);

    // Detect faces 
    std::vector<dlib::rectangle> faces = detector(cimg);
    // Find the pose of each face.
    std::vector<dlib::full_object_detection> shapes;

    for (auto& face : faces)
        shapes.push_back(pose_model(cimg, face));

    // Display it all on the screen
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(render_face_detections(shapes));
}

void stop_dlib()
{
    win.close_window();
}
*/
