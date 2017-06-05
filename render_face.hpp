#ifndef BIGVISION_RENDER_FACE_H_
#define BIGVISION_RENDER_FACE_H_

#include <dlib/image_processing/frontal_face_detector.h>
#include <opencv2/highgui/highgui.hpp>

void draw_polyline(cv::Mat &img, const dlib::full_object_detection& d, const int start, const int end,cv::Scalar color, bool isClosed = false)
{
    std::vector <cv::Point> points;
    for (int i = start; i <= end; ++i)
    {
        points.push_back(cv::Point(d.part(i).x(), d.part(i).y()));
    }
    cv::polylines(img, points, isClosed,color , 2, 16);

}

void render_face (cv::Mat &img, const dlib::full_object_detection& d)
{
    DLIB_CASSERT
    (
     d.num_parts() == 68,
     "\t std::vector<image_window::overlay_line> render_face_detections()"
     << "\n\t Invalid inputs were given to this function. "
     << "\n\t d.num_parts():  " << d.num_parts()
     );

    //  for(int i =0; i<68; i++){
    //    cv::circle(img,cv::Point(d.part(i).x(), d.part(i).y()), 5, cv::Scalar(0,255,0), -1, 8,0);
    //    cv::putText(img , std::to_string(i) , cv::Point(d.part(i).x()+3, d.part(i).y()+3),1, 1.5, cv::Scalar(0, 0,0));
    //  }

    draw_polyline(img, d, 0, 16,cv::Scalar(42,255,32));           // Jaw line
    draw_polyline(img, d, 17, 21,cv::Scalar(100,100,50));          // Left eyebrow
    draw_polyline(img, d, 22, 26, cv::Scalar(255,124,33));          // Right eyebrow
    draw_polyline(img, d, 27, 30, cv::Scalar(200,100,250));          // Nose bridge
    draw_polyline(img, d, 30, 35, cv::Scalar(120,200,10), true);    // Lower nose
    draw_polyline(img, d, 36, 41, cv::Scalar(90,50,150), true);    // Left eye
    draw_polyline(img, d, 42, 47, cv::Scalar(122,222,151),true);    // Right Eye
    draw_polyline(img, d, 48, 59, cv::Scalar(40,100,100), true);    // Outer lip
    draw_polyline(img, d, 60, 67,cv::Scalar(255,255,100), true);    // Inner lip

}

#endif // BIGVISION_RENDER_FACE_H_
