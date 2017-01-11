// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <string.h>
#include <boost/thread.hpp>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Matrix.h"
#include "Vector.h"
#include "Camera.h"
#include "pointcloud.h"
//#include "detector.h"
#include "Globals.h"
//#include "VisualisationMarkers.h"

#include "ground_plane_estimation/GroundPlane.h"
#include "upper_body_detector/UpperBodyDetector.h"

#include <QImage>
#include <QPainter>
#include <ros/package.h>

#include "reidentifier.h"
#include "AncillaryMethods.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace upper_body_detector;
using namespace ground_plane_estimation;

ros::Publisher pub_message, pub_centres, pub_closest, pub_markers, pub_features;
image_transport::Publisher pub_result_image, pub_result_imageDebug;

//VisualisationMarkers* vm;

cv::Mat img_depth_;
cv_bridge::CvImagePtr cv_depth_ptr;	// cv_bridge for depth image
cv_bridge::CvImagePtr cv_depthdisplay_ptr;	// cv_bridge for depth image to display

ReIdentifier* ridentify;

int noOfFrames = 0;

bool finished=false;

void PrintMatrix2File(cv::Mat Mat, char filename[])
{
    //FILE* pFile;
    //pFile = fopen(filename, "w");
    //CvSize Size = cvGetSize(Mat);

    ROS_INFO("image path: %s",filename);

    std::ofstream fs;
    fs.open(filename);
    if (fs.is_open())
    {
        ROS_INFO("file opened: %s",filename);
    }
    else
    {
        ROS_INFO("error openning file: %s",filename);
    }
    for (int i = 0;i<Mat.rows;i++)
    {
        for(int j = 0;j<Mat.cols;j++)
        {
            float val = Mat.at<float>(i, j);
            fs << val << " ";
//            ROS_INFO("val:%f",val);
            //CvScalar El = cvGet2D(Mat, i, j);
            //fprintf(pFile, "%f;" , El.val[0]);
            //fprintf(pFile, "%f;" , cvmGet(Mat, i, j));
        }
        // fprintf(pFile, "\n");
        fs << std::endl;
    }
    //fclose(pFile);
    fs.close();
}

void PrintMatrix2File_ushort(cv::Mat Mat, char filename[])
{
    //FILE* pFile;
    //pFile = fopen(filename, "w");
    //CvSize Size = cvGetSize(Mat);

    ROS_INFO("image path: %s",filename);

    std::ofstream fs;
    fs.open(filename);
    if (fs.is_open())
    {
        ROS_INFO("file opened: %s",filename);
    }
    else
    {
        ROS_INFO("error openning file: %s",filename);
    }
    for (int i = 0;i<Mat.rows;i++)
    {
        for(int j = 0;j<Mat.cols;j++)
        {
            int val = Mat.at<ushort>(i, j);
            fs << val << " ";
//            ROS_INFO("val:%f",val);
            //CvScalar El = cvGet2D(Mat, i, j);
            //fprintf(pFile, "%f;" , El.val[0]);
            //fprintf(pFile, "%f;" , cvmGet(Mat, i, j));
        }
        // fprintf(pFile, "\n");
        fs << std::endl;
    }
    //fclose(pFile);
    fs.close();
}

//void render_bbox_2D(UpperBodyDetector& detections, QImage& image,
//                    int r, int g, int b, int lineWidth)
//{

//    QPainter painter(&image);

//    QColor qColor;
//    qColor.setRgb(r, g, b);

//    QPen pen;
//    pen.setColor(qColor);
//    pen.setWidth(lineWidth);

//    painter.setPen(pen);

//    for(int i = 0; i < detections.pos_x.size(); i++){
//        int x =(int) detections.pos_x[i];
//        int y =(int) detections.pos_y[i];
//        int w =(int) detections.width[i];
//        int h =(int) detections.height[i];
//        ROS_INFO("detection x:%i, y:%i, w:%i, h:%i", x,y,w,h);

//        if (x+w<image.width() & y+h < image.height())
//        {
//            painter.drawLine(x,y, x+w,y);
//            painter.drawLine(x,y, x,y+h);
//            painter.drawLine(x+w,y, x+w,y+h);
//            painter.drawLine(x,y+h, x+w,y+h);
//        }
//    }
//}

void RenderBBox2DwithBiometricDistance(const vector<float> &biometricFeatures, float distance, const Vector<double>& bbox, QImage & image, int r, int g, int b)
{
    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    //// Cv:Mat
//    cv::Rect rec;
//    rec.x = x;
//    rec.y = y;
//    rec.width = w;
//    rec.height = h;

////    double score = -bbox(4);
////    stringstream score_str;
////    score_str << score;

//    const unsigned char color[] = {r,g,b};
//    cv::rectangle(image,rec,cv::Scalar(b,g,r),2);
////    cv::putText(image, score_str.str(),cv::Point(x-5,y-5),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2);
////    image.draw_rectangle_1(x,y,x+w,y+h,color,3);
////    image.draw_text(x-5,y-5,score_str.str().c_str(),1,color);

    stringstream biometric_str1,biometric_str2,biometric_str3,biometric_str4;
    biometric_str1 << "height: " << biometricFeatures[0] << "\n";
    biometric_str2 << "shoulder: " << biometricFeatures[1] << "\n";
    biometric_str3 << "face: " << biometricFeatures[2] << "\n";
    biometric_str4 << "distance: " << distance;

//    ROS_INFO("%s",biometric_str1.str().c_str());
//    ROS_INFO("%s",biometric_str2.str().c_str());
//    ROS_INFO("%s",biometric_str3.str().c_str());
//    ROS_INFO("%s",biometric_str4.str().c_str());

////    image.draw_text(x+w+5,y+5,biometric_str.str().c_str(),1,color);
//    cv::putText(image, biometric_str1.str(),cv::Point(x+w+5,y+5),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(b,g,r),2.0);
//    cv::putText(image, biometric_str2.str(),cv::Point(x+w+5,y+20),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(b,g,r),2.0);
//    cv::putText(image, biometric_str3.str(),cv::Point(x+w+5,y+35),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(b,g,r),2.0);
//    cv::putText(image, biometric_str4.str(),cv::Point(x+w+5,y+45),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(b,g,r),2.0);


    //// QImage
//    stringstream biometric_str;
//    biometric_str << "height: " << biometricFeatures[0] << endl;
//    biometric_str << "shoulder: " << biometricFeatures[1] << endl;
//    biometric_str << "face: " << biometricFeatures[2] << endl;
//    biometric_str << "distance: " << distance;

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(2);

    painter.setPen(pen);
//    QString qstr(biometric_str1.str().c_str());
//    ROS_INFO("features: %s",qstr.toStdString().c_str());
//    if (x+w<image.width() & y+h < image.height())
//    {


        painter.drawLine(x,y, x+w,y);
        painter.drawLine(x,y, x,y+h);
        painter.drawLine(x+w,y, x+w,y+h);
        painter.drawLine(x,y+h, x+w,y+h);
//        painter.drawText(x+5,y+5,"qstr");
//    }



}

void RenderRecognizedPerson(string name, const Vector<double>& bbox, cv::Mat & image, int r, int g, int b)
{
    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    const unsigned char color[] = {r,g,b};

    stringstream biometric_str;
    biometric_str << "tag: " << name << endl;

//    image.draw_text(x+5,y+h+5,biometric_str.str().c_str(),1,color);
    cv::putText(image, biometric_str.str(),cv::Point(x+5,y+h+5),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2);


}

void RenderSegmentedBlob(const vector<float>& headPt, const vector<float>& facePartPt, const vector<float>& breastPartPt, const vector<float>& bellyPartPt, const vector<vector<float> > & shoulderPts, const cv::Mat& blob, QImage & image, int r, int g, int b)
{

    const unsigned char color[] = {r,g,b};
    const unsigned char colorHead[] = {200,200,0};
    const unsigned char colorBreast[] = {100,200,0};
    const unsigned char colorBelly[] = {0,100,200};

//    writeToTXTasCSV("./data/blob_x_torender.txt",blob.col(0));
//    writeToTXTasCSV("./data/blob_y_torender.txt",blob.col(1));

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);
    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(2);

    QColor qColorHead;
    qColorHead.setRgb(colorHead[0], colorHead[1], colorHead[2]);
    QPen penHead;
    penHead.setColor(qColorHead);
    penHead.setWidth(2);

    QColor qColorBreast;
    qColorBreast.setRgb(colorBreast[0], colorBreast[1], colorBreast[2]);
    QPen penBreast;
    penBreast.setColor(qColorBreast);
    penBreast.setWidth(2);

    QColor qColorBelly;
    qColorBelly.setRgb(colorBelly[0], colorBelly[1], colorBelly[2]);
    QPen penBelly;
    penBelly.setColor(qColorBelly);
    penBelly.setWidth(2);


    painter.setPen(pen);


    for (int i=0; i<blob.rows; i++)
    {
        int x = (int) blob.at<float>(i,0);
        int y = (int) blob.at<float>(i,1);
        if (x>=shoulderPts[0][0]+5 && x<= shoulderPts[1][0]-5)
        {
            if (y>=headPt[1] && y<= facePartPt[1])
            {
                painter.setPen(penHead);
            }
            else if (y>facePartPt[1] && y<= breastPartPt[1])
            {
                painter.setPen(penBreast);
            }
            else if (y>breastPartPt[1] && y<= bellyPartPt[1])
            {
                painter.setPen(penBelly);
            }
            else
            {
                painter.setPen(pen);
            }
        }

        painter.drawPoint(x,y);

    }

}

void RenderSegmentedBlob(const cv::Mat& blob, QImage & image, int r, int g, int b)
{

    const unsigned char color[] = {r,g,b};

//    writeToTXTasCSV("./data/blob_x_torender.txt",blob.col(0));
//    writeToTXTasCSV("./data/blob_y_torender.txt",blob.col(1));
//    ROS_INFO("blob row: %i",blob.rows);
//    for (int i=0; i<blob.rows; i++)
//    {
//        int x = (int) blob.at<float>(i,0);
//        int y = (int) blob.at<float>(i,1);
//        if (Globals::DEPTH_SCALE == 1.0)
//        {
//           image.at<cv::Vec3f>(y,x)[0] = (float) b;
//           image.at<cv::Vec3f>(y,x)[1] = (float) g;
//           image.at<cv::Vec3f>(y,x)[2] = (float) r;
//        }
//        else
//        {
//            image.at<cv::Vec3b>(y,x)[0] = (uchar) b;
//            image.at<cv::Vec3b>(y,x)[1] = (uchar) g;
//            image.at<cv::Vec3b>(y,x)[2] = (uchar) r;
//        }

////        image.draw_point(x,y,color);
//    }

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(2);

    painter.setPen(pen);

    for (int i=0; i<blob.rows; i++)
    {
        int x = (int) blob.at<float>(i,0);
        int y = (int) blob.at<float>(i,1);
        painter.drawPoint(x,y);


    }


}

void RenderSkeleton2D(const vector<float>& headPt, const vector<float>& feetPt, const vector<float>& neckPt, const vector<vector<float> > & shoulderPts, QImage & image, int r, int g, int b)
{
    const unsigned char color[] = {r,g,b};

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(10);

    painter.setPen(pen);
//    QString qstr(biometric_str1.str().c_str());
//    ROS_INFO("features: %s",qstr.toStdString().c_str());



        painter.drawPoint(headPt[0],headPt[1]);
        painter.drawPoint(feetPt[0],feetPt[1]);
        painter.drawPoint(neckPt[0],neckPt[1]);
        painter.drawPoint(shoulderPts[0][0], shoulderPts[0][1]);
        painter.drawPoint(shoulderPts[1][0], shoulderPts[1][1]);

//    cv::circle(image, cv::Point(headPt[0],headPt[1]), 3, cv::Scalar(r,g,b));
//    cv::circle(image, cv::Point(feetPt[0],feetPt[1]), 3, cv::Scalar(r,g,b));
//    cv::circle(image, cv::Point(neckPt[0],neckPt[1]), 3, cv::Scalar(r,g,b));

//    cv::circle(image, cv::Point(shoulderPts[0][0], shoulderPts[0][1]), 3, cv::Scalar(r,g,b));
//    cv::circle(image, cv::Point(shoulderPts[1][0], shoulderPts[1][1]), 3, cv::Scalar(r,g,b));

//    image.draw_circle((int) headPt[0],(int) headPt[1],3,color);
//    image.draw_circle((int) feetPt[0],(int) feetPt[1],3,color);
//    image.draw_circle((int) neckPt[0],(int) neckPt[1],3,color);

//    image.draw_circle((int) shoulderPts[0][0],(int) shoulderPts[0][1],3,color);
//    image.draw_circle((int) shoulderPts[1][0],(int) shoulderPts[1][1],3,color);

}

bool checkParam(bool success, std::string param) {
    if(!success) {
        ROS_FATAL("Parameter: '%s' could not be found! Please make sure that the parameters are available on the parameter server or start with 'load_params_from_file:=true'", param.c_str());
    }
    return success;
}

bool ReadConfigParams(ros::NodeHandle n)
{
    bool success = true;

    std::string ns = ros::this_node::getName();
    ns += "/";

    //=====================================
    // Distance Range Accepted Detections
    //=====================================
    n.param(ns+"distance_range_accepted_detections", Globals::distance_range_accepted_detections, double(7.0));

    //======================================
    // ROI
    //======================================
    success = checkParam(n.getParam(ns+"inc_width_ratio", Globals::inc_width_ratio), ns+"inc_width_ratio") && success;
    success = checkParam(n.getParam(ns+"inc_height_ratio", Globals::inc_height_ratio), ns+"inc_height_ratio") && success;
    n.param(ns+"region_size_threshold", Globals::region_size_threshold, int(10));

    //======================================
    // Freespace Parameters
    //======================================
    n.param(ns+"freespace_scaleZ", Globals::freespace_scaleZ, double(20));
    n.param(ns+"freespace_scaleX", Globals::freespace_scaleX, double(20));
    n.param(ns+"freespace_minX", Globals::freespace_minX, double(-20));
    n.param(ns+"freespace_minZ", Globals::freespace_minZ, double(0));
    n.param(ns+"freespace_maxX", Globals::freespace_maxX, double(20));
    n.param(ns+"freespace_maxZ", Globals::freespace_maxZ, double(30));
    n.param(ns+"freespace_threshold", Globals::freespace_threshold, double(120));
    n.param(ns+"freespace_max_depth_to_cons", Globals::freespace_max_depth_to_cons, int(20));

    //======================================
    // Evaluation Parameters
    //======================================
    n.param(ns+"evaluation_NMS_threshold", Globals::evaluation_NMS_threshold, double(0.4));
    n.param(ns+"evaluation_NMS_threshold_LM", Globals::evaluation_NMS_threshold_LM, double(0.4));
    n.param(ns+"evaluation_NMS_threshold_Border", Globals::evaluation_NMS_threshold_Border, double(0.4));
    n.param(ns+"evaluation_inc_height_ratio", Globals::evaluation_inc_height_ratio, double(0.2));
    n.param(ns+"evaluation_stride", Globals::evaluation_stride, int(3));
    n.param(ns+"evaluation_scale_stride", Globals::evaluation_scale_stride, double(1.03));
    n.param(ns+"evaluation_nr_scales", Globals::evaluation_nr_scales, int(1));
    n.param(ns+"evaluation_inc_cropped_height", Globals::evaluation_inc_cropped_height, int(20));
    n.param(ns+"evaluation_greedy_NMS_overlap_threshold", Globals::evaluation_greedy_NMS_overlap_threshold, double(0.1));
    n.param(ns+"evaluation_greedy_NMS_threshold", Globals::evaluation_greedy_NMS_threshold, double(0.25));
    //======================================
    // World scale
    //======================================
    success = checkParam(n.getParam(ns+"WORLD_SCALE", Globals::WORLD_SCALE), ns+"WORLD_SCALE") && success;

    //======================================
    // Depth scale
    //======================================
    success = checkParam(n.getParam(ns+"DEPTH_SCALE", Globals::DEPTH_SCALE), ns+"DEPTH_SCALE") && success;

    //======================================
    // height and width of images
    //======================================
    success = checkParam(n.getParam(ns+"dImHeight", Globals::dImHeight), ns+"dImHeight") && success;
    success = checkParam(n.getParam(ns+"dImWidth", Globals::dImWidth), ns+"dImWidth") && success;

    //====================================
    // Number of Frames / offset
    //====================================
    success = checkParam(n.getParam(ns+"numberFrames", Globals::numberFrames), ns+"numberFrames") && success;
    success = checkParam(n.getParam(ns+"nOffset", Globals::nOffset), ns+"nOffset") && success;

    //====================================
    // Size of Template
    //====================================
    success = checkParam(n.getParam(ns+"template_size", Globals::template_size), ns+"template_size") && success;

    n.param(ns+"max_height", Globals::max_height, double(2.0));
    n.param(ns+"min_height", Globals::min_height, double(1.4));

    return success;
}

//visualization_msgs::MarkerArray createVisualisation(geometry_msgs::PoseArray poses, std::string target_frame) {
//    ROS_DEBUG("Creating markers");
//    visualization_msgs::MarkerArray marker_array;
//    for(int i = 0; i < poses.poses.size(); i++) {
//        geometry_msgs::Pose p = poses.poses[i];
//        marker_array.markers.push_back(vm->createMarker(target_frame, p));
//    }
//    return marker_array;
//}

void RemoveGP(const Camera& camera, PointCloudData &point_cloud)
{

    Vector<double> plane_parameters = AncillaryMethods::PlaneToCam(camera);
    double plane_par_0 = plane_parameters(0);
    double plane_par_1 = plane_parameters(1);
    double plane_par_2 = plane_parameters(2);
    double plane_par_3 = plane_parameters(3)*Globals::WORLD_SCALE;

    float freespace_max_depth_to_cons=7;

    for(int j = 0; j < point_cloud.X.getSize(); j++)
    {
        double zj = point_cloud.Z(j);

        if(zj < freespace_max_depth_to_cons && zj >= 0.1)
        {
            double xj = point_cloud.X(j);
            double yj = point_cloud.Y(j);

            double distance_to_plane = plane_par_0*xj + plane_par_1*yj + plane_par_2*zj + plane_par_3;

            // Only accept points in upper_body height range (0.1m to 2.5m)
            if(distance_to_plane > 2.5 || distance_to_plane < 0.1)
            {

                //                pcloud_GPremoved.X(j) = 0;
                //                pcloud_GPremoved.Y(j) = 0;
                point_cloud.Z(j) = 0;
            }

        }
    }


}

void callback(const ImageConstPtr &depth, const ImageConstPtr &color, const GroundPlane::ConstPtr &gp, const UpperBodyDetector::ConstPtr &detections, const CameraInfoConstPtr &info)
{

    if (noOfFrames<=35)
    {

        ROS_INFO("callback called...");
        // Check if calculation is necessary
        bool vis = pub_result_image.getNumSubscribers() > 0 || pub_markers.getNumSubscribers() > 0;
        //ROS_INFO("width:%i, height:%i",depth->width,depth->height);

        // Get depth image as matrix
        cv_depth_ptr = cv_bridge::toCvCopy(depth);
        cv_depthdisplay_ptr = cv_bridge::toCvCopy(depth);
        //    cv_bridge::cvtColor(cv_depthdisplay_ptr,"bgr8");
        img_depth_ = cv_depth_ptr->image;

        //ros::Time crntTime = ros::Time::now();
        //char filename[150];
        //sprintf(filename,"/home/scosar/Documents/Datasets/ros_test/opencv_mat/cvmat_%d-%d.txt",crntTime.sec,crntTime.nsec);
        //ROS_INFO("time:%i.%i",crntTime.sec,crntTime.nsec);
        //PrintMatrix2File_ushort(img_depth_, filename);

        // find max depth
        float max_depth = 0;
        float distance;
        for(int y = 0; y < cv_depth_ptr->image.rows; y++)
        {
            for(int x = 0; x < cv_depth_ptr->image.cols; x++)
            {
                if (Globals::DEPTH_SCALE == 1000.0)
                {
                    distance = (float) cv_depth_ptr->image.at<ushort>(y, x); // ushort (16UC1) new depth format of Kinect2
                }
                else
                {
                    distance = cv_depth_ptr->image.at<float>(y, x);
                }
                if (distance == distance)
                { // exclude NaN
                    max_depth = max(distance, max_depth);
                }


            }
        }



        cv::Mat image(cv_depth_ptr->image.rows, cv_depth_ptr->image.cols, CV_8UC3);
        //    cv_depthdisplay_ptr->image.convertTo(image,CV_8UC3);
        Matrix<double> matrix_depth(info->width, info->height);
        for(int y = 0; y < cv_depth_ptr->image.rows; y++)
        {
            for(int x = 0; x < cv_depth_ptr->image.cols; x++)
            {
                float val = 0;
                if (Globals::DEPTH_SCALE == 1.0)
                {
                    val = img_depth_.at<float>(y,x);
                    distance = cv_depth_ptr->image.at<float>(y, x);
                }
                else
                {
                    val = img_depth_.at<ushort>(y,x);
                    distance = (float) cv_depth_ptr->image.at<ushort>(y, x); // ushort (16UC1) new depth format of Kinect2
                }
                float newVal = 0;
                if (val==val)
                {
                    newVal = val / Globals::DEPTH_SCALE;
                }
                else
                {
                    newVal = 0;
                }
                matrix_depth(x, y) = newVal;

                unsigned int dist_clr = (unsigned int)(distance / max_depth * 255);
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(dist_clr, dist_clr, dist_clr);
            }
        }

        //    matrix_depth.WriteToTXT("/home/scosar/Documents/matrix_depth.txt");

        // Generate base camera
        Matrix<double> R = Eye<double>(3);
        Vector<double> t(3, 0.0);
        Matrix<double> K(3,3, (double*)&info->K[0]);
        //ROS_INFO("upper_body_detector: calibration parameters read...");

        // Get GP
        Vector<double> GP(3, (double*) &gp->n[0]);
        GP.pushBack((double) gp->d);
        //ROS_INFO("upper_body_detector: ground plane read...");
        ridentify->SetGPLevel((float)gp->d);

        // Detect upper bodies
        Camera camera(K,R,t,GP);
        //ROS_INFO("upper_body_detector: Camera object created...");
        PointCloudData point_cloud(camera, matrix_depth);

        // Convert UpperBodyDetector to Vector<Vector< double> >
        Vector<Vector< double > > detected_bounding_boxes;
        Vector<double> bbox;
        int noofPeople = detections->pos_x.size();
        for (int i=0; i<noofPeople; i++)
        {
            bbox.clearContent();
            bbox.pushBack(detections->pos_x[i]);
            bbox.pushBack(detections->pos_y[i]);
            bbox.pushBack(detections->width[i]);
            bbox.pushBack(detections->height[i]);

            detected_bounding_boxes.pushBack(bbox);
        }

        //    point_cloud.Z.writeToTXT("/home/scosar/Documents/Z_Data.txt");
        //    point_cloud.Y.writeToTXT("/home/scosar/Documents/Y_Data.txt");
        //    point_cloud.X.writeToTXT("/home/scosar/Documents/X_Data.txt");


        RemoveGP(camera,point_cloud);
        ROS_INFO("reidentifier: processing frame...");
        ROS_INFO("callback - size bbox:%i", detected_bounding_boxes.getSize());
        ridentify->ExtractFeatures(point_cloud, detected_bounding_boxes, true);


        ROS_INFO("reidentifier: feature extraction done...");


        std_msgs::Float32MultiArray personDataArray;
        //Clear array
        personDataArray.data.clear();
        std_msgs::MultiArrayLayout arrayLayout;
        std_msgs::MultiArrayDimension arrayDim1,arrayDim2;
        arrayDim1.label = "PersonID";
        arrayDim1.size = detected_bounding_boxes.getSize();
        arrayDim1.stride = 7 * detected_bounding_boxes.getSize();
        arrayDim2.label = "Features";
        arrayDim2.size = 7;
        arrayDim2.stride = 7;

        arrayLayout.dim.push_back(arrayDim1);
        arrayLayout.dim.push_back(arrayDim2);
        arrayLayout.data_offset = 0;
        personDataArray.layout = arrayLayout;

        for(int jj = 0; jj < detected_bounding_boxes.getSize(); jj++)
        {

            if (ridentify->heightPeople.size()>jj)
            {
                personDataArray.data.push_back(ridentify->heightPeople[jj]);
                personDataArray.data.push_back(ridentify->widthShoulders[jj]);
                personDataArray.data.push_back(ridentify->lengthFaces[jj]);
                personDataArray.data.push_back(0);
                personDataArray.data.push_back(ridentify->volFacePeople[jj]);
                personDataArray.data.push_back(ridentify->volBreastPeople[jj]);
                personDataArray.data.push_back(ridentify->volBellyPeople[jj]);

                noOfFrames++;

            }

        }

        pub_features.publish(personDataArray);

        // Creating a ros image with the detection results an publishing it
        if(vis) {
            //ROS_DEBUG("Publishing image");
            QImage image_rgb(&color->data[0], color->width, color->height, QImage::Format_RGB888); // would opencv be better?
            QImage image_depth(image.data, image.cols, image.rows, QImage::Format_RGB888); // would opencv be better?
            //        render_bbox_2D(detection_msg, image_rgb, 0, 0, 255, 2);

            for(int jj = 0; jj < detected_bounding_boxes.getSize(); jj++)
            {
                //            detected_bounding_boxes(jj)(3)/=3;
                //            detected_bbox = detected_bounding_boxes(jj);
                //            detected_bbox(0) = detected_bbox(0)-20;
                vector<float> biometricFeatures;
                if (ridentify->heightPeople.size()>jj)
                {
                    //                cout << "height: " << heights.at(jj) << ", shoulders: " << shoulders.at(jj) << ", faces: "  << faces.at(jj) << endl;
                    ROS_INFO_STREAM("height: " << ridentify->heightPeople[jj] << ", shoulders: " << ridentify->widthShoulders[jj] << ", faces: "  << ridentify->lengthFaces[jj] << ", volFace: " << ridentify->volFacePeople[jj]);
                    biometricFeatures.push_back(ridentify->heightPeople[jj]);
                    biometricFeatures.push_back(ridentify->widthShoulders[jj]);
                    biometricFeatures.push_back(ridentify->lengthFaces[jj]);

                    //RenderSegmentedBlob(ridentify->segmentedBlobs[jj], image_depth, 250, 0, 0 );
                    RenderSegmentedBlob(ridentify->headPtPeople[jj],ridentify->facePartPtPeople[jj],ridentify->breastPartPtPeople[jj],ridentify->bellyPartPtPeople[jj],ridentify->shoulderPtsPeople[jj],ridentify->segmentedBlobs[jj],image_depth,250,0,0);

                    RenderBBox2DwithBiometricDistance(biometricFeatures, 0, detected_bounding_boxes(jj), image_depth, 0, 0, 255);
                    RenderBBox2DwithBiometricDistance(biometricFeatures, 0, detected_bounding_boxes(jj), image_rgb, 0, 0, 255);


                    RenderSkeleton2D(ridentify->headPtPeople.at(jj),ridentify->feetPtPeople.at(jj),ridentify->neckPtPeople.at(jj),ridentify->shoulderPtsPeople.at(jj),image_depth,150,0,155);

                    //                if (pId == jj)
                    //                {
                    //                    RenderRecognizedPerson(ridentify->personName,detected_bounding_boxes[jj],image,255,0,0);
                    //                }

                }

            }

            //        cv::imwrite("/home/scosar/Documents/output.png",image);

            ROS_INFO("reidentifier: publishing image...");

            // Generate output message
            sensor_msgs::Image sensor_image, rgb_image;
            sensor_image.header = depth->header;
            sensor_image.height = image_depth.height();
            sensor_image.width  = image_depth.width();
            sensor_image.step   = sensor_image.width*3;
            vector<unsigned char> image_bits(image_depth.bits(), image_depth.bits()+sensor_image.height*sensor_image.width*3);
            sensor_image.data = image_bits;
            sensor_image.encoding = "bgr8";

            rgb_image.header = color->header;
            rgb_image.height = image_rgb.height();
            rgb_image.width  = image_rgb.width();
            rgb_image.step   = rgb_image.width*3;
            vector<unsigned char> image_bits_rgb(image_rgb.bits(), image_rgb.bits()+rgb_image.height*rgb_image.width*3);
            rgb_image.data = image_bits_rgb;
            rgb_image.encoding = "bgr8";

            //        cv_depthdisplay_ptr->image = image;

            //        pub_result_image.publish(cv_depthdisplay_ptr->toImageMsg());
            pub_result_image.publish(rgb_image);
            pub_result_imageDebug.publish(sensor_image);
        }



    }
    else
    {
        if (finished==false)
        {
            ridentify->LearnModels(0);
            ROS_INFO("Enter the name of the person: ");
            cin >> ridentify->personName;
            ridentify->saveLearnedModel(ros::package::getPath("reidentifier") + "/config/biometricFeatures.yaml");

            finished=true;
            //        ros::shutdown();

            ROS_INFO("Training is done! Please close the software by pressing CTRL+C");
        }

    }


}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<CameraInfo> &sub_cam,
                     message_filters::Subscriber<UpperBodyDetector> &sub_det,
                     message_filters::Subscriber<GroundPlane> &sub_gp,
                     image_transport::SubscriberFilter &sub_dep,
                     image_transport::SubscriberFilter &sub_col,
                     image_transport::ImageTransport &it){
    if(!pub_message.getNumSubscribers()
            && !pub_result_imageDebug.getNumSubscribers()) {
        ROS_DEBUG("Reidentifier: No subscribers. Unsubscribing.");
        sub_cam.unsubscribe();
        sub_gp.unsubscribe();
        sub_det.unsubscribe();
        sub_col.unsubscribe();
        sub_dep.unsubscribe();
    } else {
        ROS_DEBUG("Reidentifier: New subscribers. Subscribing.");
        sub_cam.subscribe();
        sub_gp.subscribe();
        sub_det.subscribe();
        sub_dep.subscribe(it,sub_dep.getTopic().c_str(),1);
        sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    }
}

int main(int argc, char **argv)
{

    // Set up ROS.
    ros::init(argc, argv, "reidentifier");
    ros::NodeHandle n;

//    vm = new VisualisationMarkers();

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string cam_ns;
    string config_file;
    string model_path;
    string topic_gp;
    string topic_det;

    string pub_topic_centres;
    string pub_topic_closest;
    string pub_topic_ri;
    string pub_topic_result_image;
    string pub_topic_result_imageDebug;
    string pub_topic_features;
    string pub_markers_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(5));
    private_node_handle_.param("config_file", config_file, string(""));
//    private_node_handle_.param("model_file", model_path, string(""));

    private_node_handle_.param("camera_namespace", cam_ns, string("/camera"));
    private_node_handle_.param("ground_plane", topic_gp, string("/ground_plane"));
    private_node_handle_.param("upper_body_detector", topic_det, string("/upper_body_detector/detections"));

    std::vector<double> meanVector,icovMatrix;
    int meanVector_rows, meanVector_cols, icovMatrix_rows, icovMatrix_cols;
    string personName;
//    private_node_handle_.getParam("meanVector_rows", meanVector_rows);
//    private_node_handle_.getParam("meanVector_cols", meanVector_cols);
//    private_node_handle_.getParam("icovMatrix_rows", icovMatrix_rows);
//    private_node_handle_.getParam("icovMatrix_cols", icovMatrix_cols);
//    private_node_handle_.getParam("name", personName);

//    if(!checkParam(private_node_handle_.getParam("meanVector_data", meanVector), ros::this_node::getName()+"/meanVector")){
//        ROS_FATAL("No mean vector found");
//        return 1;
//    }
//    if(!checkParam(private_node_handle_.getParam("icovMatrix_data", icovMatrix), ros::this_node::getName()+"/icovMatrix")){
//        ROS_FATAL("No inverse covariance matrix found");
//        return 1;
//    }

    string topic_color_image;
    private_node_handle_.param("rgb_image", topic_color_image, string("/rgb/image_rect_color"));
    topic_color_image = cam_ns + topic_color_image;
    string topic_camera_info;
    private_node_handle_.param("camera_info_depth", topic_camera_info, string("/depth/camera_info"));
    topic_camera_info = cam_ns + topic_camera_info;
    string topic_depth_image;
    private_node_handle_.param("depth_image", topic_depth_image, string("/depth/image_rect_meters"));
    topic_depth_image = cam_ns + topic_depth_image;

    // Printing queue size
    ROS_DEBUG("reidentifier: Queue size for synchronisation is set to: %i", queue_size);

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    image_transport::SubscriberFilter subscriber_depth;
    subscriber_depth.subscribe(it, topic_depth_image.c_str(),1); subscriber_depth.unsubscribe();
    message_filters::Subscriber<CameraInfo> subscriber_camera_info(n, topic_camera_info.c_str(), 1); subscriber_camera_info.unsubscribe();
    image_transport::SubscriberFilter subscriber_color;
    subscriber_color.subscribe(it, topic_color_image.c_str(), 1); subscriber_color.unsubscribe();
    message_filters::Subscriber<GroundPlane> subscriber_gp(n, topic_gp.c_str(), 1); subscriber_gp.unsubscribe();
    message_filters::Subscriber<UpperBodyDetector> subscriber_det(n, topic_det.c_str(), 1); subscriber_det.unsubscribe();


    ROS_INFO("calling callback...");
    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(subscriber_camera_info),
                                                       boost::ref(subscriber_det),
                                                       boost::ref(subscriber_gp),
                                                       boost::ref(subscriber_depth),
                                                       boost::ref(subscriber_color),
                                                       boost::ref(it));
    image_transport::SubscriberStatusCallback image_cb = boost::bind(&connectCallback,
                                                                     boost::ref(subscriber_camera_info),
                                                                     boost::ref(subscriber_det),
                                                                     boost::ref(subscriber_gp),
                                                                     boost::ref(subscriber_depth),
                                                                     boost::ref(subscriber_color),
                                                                     boost::ref(it));

    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<Image, Image, GroundPlane, UpperBodyDetector, CameraInfo> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    // Initialise detector
    if(!ReadConfigParams(boost::ref(n))) return 1;

    ridentify = new ReIdentifier();


    //ROS_INFO("Upper body template read...");

    // Create synchronization policy. Here: async because time stamps will never match exactly
    const sync_policies::ApproximateTime<Image, Image, GroundPlane, UpperBodyDetector, CameraInfo> MyConstSyncPolicy = MySyncPolicy;
    Synchronizer< sync_policies::ApproximateTime<Image, Image, GroundPlane, UpperBodyDetector, CameraInfo> > sync(MyConstSyncPolicy,
                                                                                               subscriber_depth,
                                                                                               subscriber_color,
                                                                                               subscriber_gp,
                                                                                               subscriber_det,
                                                                                               subscriber_camera_info);


    // Register one callback for all topics
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

    // Create publisher

    private_node_handle_.param("reidentifier_features", pub_topic_features, string("/reidentifier/features"));
    pub_features = n.advertise<std_msgs::Float32MultiArray>(pub_topic_features.c_str(), 10, con_cb, con_cb);

//    private_node_handle_.param("upper_body_bb_centres", pub_topic_centres, string("/upper_body_detector/bounding_box_centres"));
//    pub_centres = n.advertise<geometry_msgs::PoseArray>(pub_topic_centres.c_str(), 10, con_cb, con_cb);

//    private_node_handle_.param("upper_body_closest_bb_centres", pub_topic_closest, string("/upper_body_detector/closest_bounding_box_centre"));
//    pub_closest = n.advertise<geometry_msgs::PoseStamped>(pub_topic_closest.c_str(), 10, con_cb, con_cb);

//    private_node_handle_.param("upper_body_markers", pub_markers_topic, string("/upper_body_detector/marker_array"));
//    pub_markers = n.advertise<visualization_msgs::MarkerArray>(pub_markers_topic.c_str(), 10, con_cb, con_cb);

    private_node_handle_.param("reidentifier_imageDebug", pub_topic_result_imageDebug, string("/reidentifier/imageDebug"));
    pub_result_imageDebug = it.advertise(pub_topic_result_imageDebug.c_str(), 1, image_cb, image_cb);

    private_node_handle_.param("reidentifier_image", pub_topic_result_image, string("/reidentifier/image"));
    pub_result_image = it.advertise(pub_topic_result_image.c_str(), 1, image_cb, image_cb);

    // Start ros thread managment
//    if (finished)
//    {
//        return 0;
//    }
//    else
//    {
//    while (ros::ok() && !finished)
//    {
//        ros::spinOnce();
//    }
//    }

    ros::spin();

    return 0;
}


