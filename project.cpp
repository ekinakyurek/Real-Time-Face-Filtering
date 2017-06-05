#include <dlib/opencv.h>
#include <dlib/iosockstream.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include "render_face.hpp"
#include <string>
#include <sstream>
#include <math.h>

using namespace dlib;
using namespace std;

#define FACE_DOWNSAMPLE_RATIO 4
#define SKIP_FRAMES 2
#define SKIP_DATA_TRANSFER 1
#define OPENCV_FACE_RENDER
//Debug mode enables to see landmark detection. Turn on this for debugging.
#define DEBUG_MODE
// If it is in the unity mode it tries to connect the socket. Turn off this for debugging.
//#define UNITY_MODE

//For Macbook Pro Camera
std::vector<cv::Point3d> get_3d_model_points()
{
    std::vector<cv::Point3d> modelPoints;

    modelPoints.push_back(cv::Point3d(0.0f, 0.0f, 0.0f)); //The first must be (0,0,0) while using POSIT
    modelPoints.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));
    modelPoints.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));
    modelPoints.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));
    modelPoints.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));
    modelPoints.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));

    return modelPoints;

}

//Head pose estimation points

std::vector<cv::Point2d> get_2d_image_points(full_object_detection &d)
{
    std::vector<cv::Point2d> image_points;
    image_points.push_back( cv::Point2d( d.part(30).x(), d.part(30).y() ) );    // Nose tip
    image_points.push_back( cv::Point2d( d.part(8).x(), d.part(8).y() ) );      // Chin
    image_points.push_back( cv::Point2d( d.part(36).x(), d.part(36).y() ) );    // Left eye left corner
    image_points.push_back( cv::Point2d( d.part(45).x(), d.part(45).y() ) );    // Right eye right corner
    image_points.push_back( cv::Point2d( d.part(48).x(), d.part(48).y() ) );    // Left Mouth corner
    image_points.push_back( cv::Point2d( d.part(54).x(), d.part(54).y() ) );    // Right mouth corner
    return image_points;

}

//To put glass on the nose bridge
cv::Point2d get_nose_bride(full_object_detection &d)
{
    return cv::Point2d((d.part(27).x()+d.part(27).x())/2.0, (d.part(27).y()+d.part(27).y())/2.0);    // Nose bridge
}

cv::Mat get_camera_matrix(float focal_length, cv::Point2d center)
{
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    return camera_matrix;
}

bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

//Calculating Eular angles for Unity from Rotation Matrix
std::vector<double> rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    std::vector<double> v(3);
    v[0] = (x/M_PI)*180.0;
    v[1] = (y/M_PI)*180.0;
    v[2] = (z/M_PI)*180.0;
    return v;
}

int main()
{

   cout << "OpenCV version : " << CV_VERSION << endl;

    try
    {

      #ifdef UNITY_MODE
        //connect to the socket
        iosockstream stream("localhost:1234");
      #endif
        //start opencv camera capture
        cv::VideoCapture cap(0);
        if (!cap.isOpened())
        {
            cerr << "Unable to connect to camera" << endl;
            return 1;
        }

        double fps = 30.0; // Just a place holder. Actual value calculated after 100 frames.
        cv::Mat im;

        // Get first frame and allocate memory.
        //get frame
        cap >> im;
        cv::Mat im_small, im_display;
        //resizing imade for face detection
        cv::resize(im, im_small, cv::Size(), 1.0/FACE_DOWNSAMPLE_RATIO, 1.0/FACE_DOWNSAMPLE_RATIO);
        cv::resize(im, im_display, cv::Size(), 0.5, 0.5);

        cv::Size size = im.size();



#ifndef OPENCV_FACE_RENDER
        //open X11 windows
        image_window win;
#endif

        // Load face detection and pose estimation models.
        frontal_face_detector detector = get_frontal_face_detector();
        //Load article's model from dat file
        shape_predictor pose_model;
        deserialize("shape_predictor_68_face_landmarks.dat") >> pose_model;

        //count for frames
        int count = 0;
        std::vector<rectangle> faces;
        // Grab and process frames until the main window is closed by the user.
        double t = (double)cv::getTickCount();
#ifdef OPENCV_FACE_RENDER
        while(1)
#else
        while(!win.is_closed())
#endif
        {

            if ( count == 0 )
                t = cv::getTickCount();
            // Grab a frame
            cap >> im;

            // Resize image for face detection
            cv::resize(im, im_small, cv::Size(), 1.0/FACE_DOWNSAMPLE_RATIO, 1.0/FACE_DOWNSAMPLE_RATIO);

            // Change to dlib's image format. No memory is copied.
            cv_image<bgr_pixel> cimg_small(im_small);
            cv_image<bgr_pixel> cimg(im);


            // Detect faces in every two frames
            if ( count % SKIP_FRAMES == 0 )
            {
                faces = detector(cimg_small);
            }

            // Pose estimation
            std::vector<cv::Point3d> model_points = get_3d_model_points();


            // Find the pose of each face.
            std::vector<full_object_detection> shapes;

            //Get face coordinates
            for (unsigned long i = 0; i < faces.size(); ++i)
            {
                rectangle r(
                            (long)(faces[i].left() * FACE_DOWNSAMPLE_RATIO),
                            (long)(faces[i].top() * FACE_DOWNSAMPLE_RATIO),
                            (long)(faces[i].right() * FACE_DOWNSAMPLE_RATIO),
                            (long)(faces[i].bottom() * FACE_DOWNSAMPLE_RATIO)
                            );

                //get landmark points
                full_object_detection shape = pose_model(cimg, r);
                //push landmark points for each face
                shapes.push_back(shape);
#ifdef OPENCV_FACE_RENDER
                //render polylines
                render_face(im, shape);
                //get important landmark points
                std::vector<cv::Point2d> image_points = get_2d_image_points(shape);

                double focal_length = im.cols;
                //get camera matrix
                cv::Mat camera_matrix = get_camera_matrix(focal_length, cv::Point2d(im.cols/2,im.rows/2));
                cv::Mat rotation_vector;
                cv::Mat rotation_matrix;
                cv::Mat translation_vector;

                //create dist_coefficents
                cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);

                //solver for rotation and traslational vectors
                cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
                std::vector<cv::Point3d> nose_end_point3D;
              	std::vector<cv::Point2d> nose_end_point2D;
              	nose_end_point3D.push_back(cv::Point3d(0,0,1000.0));

                //project points to 2D
              	cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);

                //Plot pose line
              	cv::line(im,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);





                #ifdef UNITY_MODE
                //a is the data will be sent
                  if(count % (SKIP_DATA_TRANSFER) == 0){
                    cv::Point2d nose_bridge = get_nose_bride(shape);
                    string a = "";
                /**Format of a
                      [left,top,right,bottom],[(nose tip), (Chin), (left eye corner) , (right eye corner), (left mouth corner) , (right moutcorner)],[nose bridge],[euler angles]
                 */
                    a = a + "[" + to_string((long)(faces[i].left() * FACE_DOWNSAMPLE_RATIO))+ "," +to_string((long)(faces[i].top() * FACE_DOWNSAMPLE_RATIO))+ ","+to_string((long)(faces[i].right() * FACE_DOWNSAMPLE_RATIO))+","+  to_string((long)(faces[i].bottom() * FACE_DOWNSAMPLE_RATIO))+"]";
                    a = a + ",[";
                    for(unsigned int i = 0 ; i < image_points.size(); i++){
                          a = a + "(" + to_string(image_points[i].x) + "," + to_string(image_points[i].y) + "),";
                    }
                    //trimming
                    a.pop_back();
                    a = a + "],[" + to_string(nose_bridge.x) + "," + to_string(nose_bridge.y) + "]";
                    cv::Rodrigues(rotation_vector, rotation_matrix);
                    std::vector<double> rotation_angles = rotationMatrixToEulerAngles(rotation_matrix);
                    a = a + ",[" + to_string(rotation_angles[0]) + "," + to_string(rotation_angles[1]) + ","+ to_string(rotation_angles[2]) + "]";
                    stream << a + "\n";
                 }
                #else
                  //cout << a + "\n" ;
                #endif
//                cv::Point2d projected_point = find_projected_point(rotation_matrix, translation_vector, camera_matrix, cv::Point3d(0,0,1000.0));
							//	cv::line(im,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
//                cv::line(im,image_points[0], projected_point, cv::Scalar(0,0,255), 2);




#endif
            }
        		// Uncomment the line below to see FPS
            //cv::putText(im, cv::format("fps %.2f",fps), cv::Point(50, size.height - 50), cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);


            // Display it all on the screen
#ifdef OPENCV_FACE_RENDER

                // Resize image for display
      #ifdef DEBUG_MODE
               im_display = im;
               cv::resize(im, im_display, cv::Size(), 0.5, 0.5);
               cv::imshow("Fast Facial Landmark Detector", im_display);

              //  WaitKey slows down the runtime quite a lot
              //  So check every 15 frames


                if ( count % 15 == 0)
                {
                    int k = cv::waitKey(1);
                    // Quit if 'q' or ESC is pressed
                    if ( k == 'q' || k == 27)
                    {
                        return 0;
                    }
                }
      #endif



#else

                win.clear_overlay();
                win.set_image(cimg);
                win.add_overlay(render_face_detections(shapes));
#endif

            count++;

            if ( count == 100)
            {
                t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
                fps = 100.0/t;
                count = 0;
            }



        }
    }
    catch(serialization_error& e)
    {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
    }
    catch(exception& e)
    {
        cout << e.what() << endl;
    }
}
