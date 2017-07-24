// Optris
#include <libirimager/ImageBuilder.h>
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// GSL
#include <gsl/gsl_interp.h>
// C++ STD
#include <fstream>
#include <numeric>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "physiological_monitoring/PhysiologicalData.h"
#include "st_anomaly_detector/StringArrayStamped.h"

// Optris
evo::ImageBuilder image_builder_;
image_transport::Publisher biometrics_pub_;
ros::Publisher pub_phy, pub_phy_level;
unsigned char* thermal_buffer_ = NULL;
double** temperature_map_;

// Respiration rate parameters
double* respiration_buffer_;
double* respiration_times_;
int rb_size_ = 0;

// Heart rate parameters
double* heart_buffer_;
double* heart_times_;
int hb_size_ = 0;

// ROS parameters:
double temperature_min_;
double temperature_max_;

double respiration_rate_min_;
double respiration_rate_max_;
int respiration_buffer_min_;
int respiration_buffer_max_;

double heart_rate_min_;
double heart_rate_max_;
int heart_buffer_min_;
int heart_buffer_max_;

int contour_area_min_;
double temperature_low_thr_;
double temperature_high_thr_;
double respiration_low_thr_;
double respiration_high_thr_;
double heart_low_thr_;
double heart_high_thr_;

bool data_visualization_;

std::string pub_topic_phy, pub_topic_phylevel;

// Face detection
cv::Mat binary_image_;

// FPS
clock_t start_time_;
double fps_times_[11];
int fps_size_ = 0;

void colorConvert(const sensor_msgs::ImageConstPtr& raw_image, sensor_msgs::Image& color_image) {
  unsigned short* data = (unsigned short*)&raw_image->data[0];
  image_builder_.setData(raw_image->width, raw_image->height, data);
  
  if(thermal_buffer_ == NULL)
    thermal_buffer_ = new unsigned char[raw_image->width * raw_image->height * 3];
  
  image_builder_.convertTemperatureToPaletteImage(thermal_buffer_, true);
  
  color_image.header.frame_id = "physiological_monitoring";
  color_image.height          = raw_image->height;
  color_image.width           = raw_image->width;
  color_image.encoding        = "rgb8";
  color_image.step            = raw_image->width * 3;
  color_image.header.seq      = raw_image->header.seq;
  color_image.header.stamp    = ros::Time::now();
  
  color_image.data.resize(color_image.height * color_image.step);
  memcpy(&color_image.data[0], &thermal_buffer_[0], color_image.height * color_image.step * sizeof(*thermal_buffer_));
}

double bioTemperature(cv::Mat& cv_image, cv::Rect& face) {
  //std::cerr << "x = " << face.x << ", y = " << face.y << ", width = " << face.width << ", height = " << face.height << std::endl;
  double t_sum = 0, t_cnt = 0, t_avg = 0;
  for(int i = face.y; i < int(face.y+face.height); i++) {
    if(i >= 0 && i < cv_image.rows) {
      for(int j = face.x; j < int(face.x+face.width); j++) {
	if(j >= 0 && j < cv_image.cols) {
	  if(temperature_map_[i][j] > temperature_min_ && temperature_map_[i][j] < temperature_max_) {
	    t_sum += temperature_map_[i][j];
	    t_cnt++;
	  }
	}
      }
    }
  }
  if(t_cnt > 0) t_avg = t_sum / t_cnt;
  //std::cerr << "t_cnt = " << t_cnt << std::endl;
  
  std::ostringstream ss;
  ss << round(t_avg*10)/10.0;
  std::string t_avg_string("Temp:" + ss.str()+"c");
  putText(cv_image, t_avg_string, cv::Point(face.x+face.width*0.5+30, face.y-20), 0, 0.6, cv::Scalar(255, 255, 255));
  return t_avg;
}

double bioRespirationRate(cv::Mat& cv_image, cv::Rect& nose) {
  double t_sum = 0, t_cnt = 0;
  for(int i = nose.y; i < int(nose.y+nose.height); i++) {
    if(i >= 0 && i < cv_image.rows) {
      for(int j = nose.x; j < int(nose.x+nose.width); j++) {
	if(j >= 0 && j < cv_image.cols) {
	  t_sum += temperature_map_[i][j];
	  t_cnt++;
	}
      }
    }
  }
  if(t_cnt == 0)
    return -1;
  
  // FIFO buffer.
  if(rb_size_ == respiration_buffer_max_) {
    for(int i = 0; i < rb_size_-1; i++) {
      respiration_buffer_[i] = respiration_buffer_[i+1];
      respiration_times_[i] = respiration_times_[i+1];
    }
    rb_size_--;
  }
  respiration_buffer_[rb_size_] = t_sum / t_cnt;
  respiration_times_[rb_size_] = double(clock() - start_time_) / CLOCKS_PER_SEC;
  rb_size_++;
  
  double resprate = -1;
  if(rb_size_ > respiration_buffer_min_) {
    // Evenly spaced numbers over a specified interval.
    double even_times[rb_size_];
    double delta = (respiration_times_[rb_size_-1]-respiration_times_[0]) / double(rb_size_-1);
    for(int i = 0; i < rb_size_-1; i++)
      even_times[i] = respiration_times_[0] + delta * i;
    even_times[rb_size_-1] = respiration_times_[rb_size_-1];
    // One-dimensional linear interpolation.
    gsl_interp *interpolation = gsl_interp_alloc(gsl_interp_linear, rb_size_);
    gsl_interp_init(interpolation, respiration_times_, respiration_buffer_, rb_size_);
    gsl_interp_accel *accelerator = gsl_interp_accel_alloc();
    double interpolated[rb_size_];
    for(int i = 0; i < rb_size_; i++) {
      interpolated[i] = gsl_interp_eval(interpolation, respiration_times_, respiration_buffer_, even_times[i], accelerator);
      // Hamming window.
      interpolated[i] *= (0.54 - 0.46 * cos(2.0 * M_PI * i / (rb_size_ - 1)));
    }
    gsl_interp_free(interpolation);
    // Normalization.
    double mean = std::accumulate(interpolated, interpolated+rb_size_, 0) / double(rb_size_);
    for(int i = 0; i < rb_size_; i++)
      interpolated[i] -= mean;
    // rfft
    std::vector<double> fft_in(interpolated, interpolated + sizeof(interpolated) / sizeof(double));
    std::vector<double> fft_out;
    cv::dft(fft_in, fft_out);
    double fft[rb_size_/2+1];
    fft[0] = fft_out[0]*fft_out[0];
    for(int i = 1, j = 1; i < rb_size_-1; i+=2, j++)
      fft[j] = fft_out[i]*fft_out[i]+fft_out[i+1]*fft_out[i+1];
    if(rb_size_%2 == 0)
      fft[rb_size_/2] = fft_out[rb_size_-1]*fft_out[rb_size_-1];
    // Respiration rate.
    double fps = double(rb_size_) / (respiration_times_[rb_size_-1]-respiration_times_[0]);
    double frequencies[rb_size_/2+1];
    double val_max = -DBL_MAX;
    int idx_max = -1;
    for(int i = 0; i < (rb_size_/2+1); i++) {
      frequencies[i] = fps / double(rb_size_) * double(i) * 60.0;
      if(frequencies[i] > respiration_rate_min_ && frequencies[i] < respiration_rate_max_) {
    	if(fft[i] > val_max) {
    	  val_max = fft[i];
    	  idx_max = i;
    	}
      }
    }
    //std::cout << "[respiration rate] frame = " << rb_size_ << ", idx_max = " << idx_max << ", bpm = " << frequencies[idx_max] << std::endl;
    if(idx_max > 0) {
      std::ostringstream ss;
      resprate = round(frequencies[idx_max]*10)/10.0;
      ss << round(frequencies[idx_max]*10)/10.0;
      putText(cv_image, "Resp:" + ss.str()+"bpm", cv::Point(nose.x+nose.width*0.5+40, nose.y-10), 0, 0.6, cv::Scalar(0, 255, 0));
    }
  }
  
//  rectangle(cv_image, cv::Rect(cv::Point(cv_image.cols-100, 1), cv::Point(cv_image.cols-1, 50)), cv::Scalar(0, 0, 0));
//  putText(cv_image, "@todo plot", cv::Point(cv_image.cols-90, 20), 4, 0.4, cv::Scalar(0, 0, 0));
  
  if(data_visualization_) {
    //for(int i = 0; i < rb_size_; i++) std::cerr << respiration_buffer_[i] << " "; std::cerr << std::endl;
    std::fstream fs;
    rb_size_ == 1 ? fs.open("respiration", std::fstream::out | std::fstream::trunc) : fs.open("respiration", std::fstream::out | std::fstream::app);
    fs << rb_size_-1 << " " << respiration_buffer_[rb_size_-1] << "\n";
    fs.close();
  }

  return resprate;
}

double bioHeartRate(cv::Mat& cv_image, cv::Rect& forehead) {
  double t_sum = 0, t_cnt = 0;
  for(int i = forehead.y; i < int(forehead.y+forehead.height); i++) {
    if(i >= 0 && i < cv_image.rows) {
      for(int j = forehead.x; j < int(forehead.x+forehead.width); j++) {
	if(j >= 0 && j < cv_image.cols) {
	  t_sum += temperature_map_[i][j];
	  t_cnt++;
	}
      }
    }
  }
  if(t_cnt == 0)
    return -1;
  
  // FIFO buffer.
  if(hb_size_ == heart_buffer_max_) {
    for(int i = 0; i < hb_size_-1; i++) {
      heart_buffer_[i] = heart_buffer_[i+1];
      heart_times_[i] = heart_times_[i+1];
    }
    hb_size_--;
  }
  heart_buffer_[hb_size_] = t_sum / t_cnt;
  heart_times_[hb_size_] = double(clock() - start_time_) / CLOCKS_PER_SEC;
  hb_size_++;
  
  double heartrate = -1;
  if(hb_size_ > heart_buffer_min_) {
    // Evenly spaced numbers over a specified interval.
    double even_times[hb_size_];
    double delta = (heart_times_[hb_size_-1]-heart_times_[0]) / double(hb_size_-1);
    for(int i = 0; i < hb_size_-1; i++)
      even_times[i] = heart_times_[0] + delta * i;
    even_times[hb_size_-1] = heart_times_[hb_size_-1];
    // One-dimensional linear interpolation.
    gsl_interp *interpolation = gsl_interp_alloc(gsl_interp_linear, hb_size_);
    gsl_interp_init(interpolation, heart_times_, heart_buffer_, hb_size_);
    gsl_interp_accel *accelerator = gsl_interp_accel_alloc();
    double interpolated[hb_size_];
    for(int i = 0; i < hb_size_; i++) {
      interpolated[i] = gsl_interp_eval(interpolation, heart_times_, heart_buffer_, even_times[i], accelerator);
      // Hamming window.
      interpolated[i] *= (0.54 - 0.46 * cos(2.0 * M_PI * i / (hb_size_ - 1)));
    }
    gsl_interp_free(interpolation);
    // Normalization.
    double mean = std::accumulate(interpolated, interpolated+hb_size_, 0) / double(hb_size_);
    for(int i = 0; i < hb_size_; i++)
      interpolated[i] -= mean;
    // rfft
    std::vector<double> fft_in(interpolated, interpolated + sizeof(interpolated) / sizeof(double));
    std::vector<double> fft_out;
    cv::dft(fft_in, fft_out);
    double fft[hb_size_/2+1];
    fft[0] = fft_out[0]*fft_out[0];
    for(int i = 1, j = 1; i < hb_size_-1; i+=2, j++)
      fft[j] = fft_out[i]*fft_out[i]+fft_out[i+1]*fft_out[i+1];
    if(hb_size_%2 == 0)
      fft[hb_size_/2] = fft_out[hb_size_-1]*fft_out[hb_size_-1];
    // Heart rate.
    double fps = double(hb_size_) / (heart_times_[hb_size_-1]-heart_times_[0]);
    double frequencies[hb_size_/2+1];
    double val_max = -DBL_MAX;
    int idx_max = -1;
    for(int i = 0; i < (hb_size_/2+1); i++) {
      frequencies[i] = fps / double(hb_size_) * double(i) * 60.0;
      if(frequencies[i] > heart_rate_min_ && frequencies[i] < heart_rate_max_) {
    	if(fft[i] > val_max) {
    	  val_max = fft[i];
    	  idx_max = i;
    	}
      }
    }
    //std::cout << "[heart rate] frame = " << hb_size_ << ", idx_max = " << idx_max << ", bpm = " << frequencies[idx_max] << std::endl;
    if(idx_max > 0) {
      std::ostringstream ss;
      heartrate = round(frequencies[idx_max]*10)/10.0;
      ss << round(frequencies[idx_max]*10)/10.0;
      putText(cv_image, "Heartbeat:" + ss.str()+"bpm", cv::Point(forehead.x+forehead.width*0.5+20, forehead.y-10), 0, 0.6, cv::Scalar(255, 191, 0));
    }
  }

  return heartrate;
}

double* faceDetection(cv::Mat cv_image) {
  if(fps_size_ == 11) {
    for(int i = 0; i < 10; i++)
      fps_times_[i] = fps_times_[i+1];
    fps_size_ = 10;
  }
  fps_times_[fps_size_] = double(clock() - start_time_) / CLOCKS_PER_SEC;
  fps_size_++;

  double phyData[3];
  phyData[0] = -1;
  phyData[1] = -1;
  phyData[2] = -1;
  
  bool just_debug = false;
  if(just_debug) {
    // Zhi
    cv::Rect face(cv::Point(130, 60), cv::Point(270, 230));
    cv::Rect nose(cv::Point(185, 165), cv::Point(225, 190));
    cv::Rect forehead(cv::Point(185, 90), cv::Point(225, 120));
    
    // Joao
    // cv::Rect face(cv::Point(140, 120), cv::Point(215, 220));
    // cv::Rect nose(cv::Point(170, 165), cv::Point(195, 180));
    // cv::Rect forehead(cv::Point(170, 125), cv::Point(190, 140));
    
    // Jaime
    // cv::Rect face(cv::Point(165, 110), cv::Point(250, 220));
    // cv::Rect nose(cv::Point(190, 165), cv::Point(220, 190));
    // cv::Rect forehead(cv::Point(190, 125), cv::Point(210, 140));
    
    // Anestis
    // cv::Rect face(cv::Point(160, 110), cv::Point(245, 220));
    // cv::Rect nose(cv::Point(185, 175), cv::Point(215, 195));
    // cv::Rect forehead(cv::Point(190, 135), cv::Point(210, 150));
    
    rectangle(cv_image, face, cv::Scalar(255, 255, 255));
    rectangle(cv_image, nose, cv::Scalar(0, 255, 0));
    rectangle(cv_image, forehead, cv::Scalar(255, 191, 0));
    
    bioTemperature(cv_image, face);
    bioRespirationRate(cv_image, nose);
    bioHeartRate(cv_image, forehead);
  } else {
    if(binary_image_.empty())
      binary_image_.create(cv_image.rows, cv_image.cols, CV_8UC1);
    
    for(int i = 0; i < cv_image.rows; i++) {
      for(int j = 0; j < cv_image.cols; j++) {
    	if(temperature_map_[i][j] > temperature_min_ && temperature_map_[i][j] < temperature_max_)
	  binary_image_.at<uchar>(i, j) = 255;
	else
	  binary_image_.at<uchar>(i, j) = 0;
      }
    }
    
    // Optional: morphological transformation (very useful for Jaime's face).
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20), cv::Point(1, 1)); // @todo ros_param: erosion_size_
    cv::morphologyEx(binary_image_, binary_image_, cv::MORPH_CLOSE, element);
    
    std::vector<std::vector<cv::Point> > contours;
    findContours(binary_image_.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int contour_area_max = 30, contour_idx = -1; // @todo ros_param
    for(int i = 0; i < contours.size(); i++) {
      double a = contourArea(contours[i], false);
      if (a > contour_area_min_) {
        if(a > contour_area_max) {
  	  contour_area_max = a;
	  contour_idx = i;
        }
      }
    }
    if(contour_idx > -1) {
        drawContours(cv_image, contours, contour_idx, cv::Scalar(0, 255, 0));

        std::vector<int> hull_ids;
        convexHull(contours[contour_idx], hull_ids);
        for(int i = 0; i < hull_ids.size(); i++)
            circle(cv_image, contours[contour_idx][hull_ids[i]], 3, cv::Scalar(255, 0, 0), 2);
        std::vector<cv::Vec4i> defects;
        cv::convexityDefects(contours[contour_idx], hull_ids, defects);
        int y_max = 0;
        std::vector<cv::Point> face_points;
        for(std::vector<cv::Vec4i>::iterator it = defects.begin(); it != defects.end(); it++) {
            if((*it)[3]/256 > 20) { // @todo ros_param: max_defect_depth_
                if(y_max < contours[contour_idx][(*it)[2]].y)
                    y_max = contours[contour_idx][(*it)[2]].y;
                face_points.push_back(contours[contour_idx][(*it)[2]]);
                circle(cv_image, contours[contour_idx][(*it)[2]], 3, cv::Scalar(0, 0, 255), 2);
            }
        }
        if(face_points.size() > 0) {
            for(int i = 0; i < hull_ids.size(); i++) {
                if(contours[contour_idx][hull_ids[i]].y < y_max)
                    face_points.push_back(contours[contour_idx][hull_ids[i]]);
            }
            cv::Rect face_box = boundingRect(face_points);
            cv::Rect nose(cv::Point(face_box.x+face_box.width*0.3, face_box.y+face_box.height*0.5),
                          cv::Point(face_box.x+face_box.width*0.7, face_box.y+face_box.height*0.8));
            cv::Rect forehead(cv::Point(face_box.x+face_box.width*0.3, face_box.y+face_box.height*0.15),
                              cv::Point(face_box.x+face_box.width*0.7, face_box.y+face_box.height*0.35));

            rectangle(cv_image, face_box, cv::Scalar(255, 255, 255));
            rectangle(cv_image, nose, cv::Scalar(0, 255, 0));
            rectangle(cv_image, forehead, cv::Scalar(255, 191, 0));

            double temperature = bioTemperature(cv_image, face_box);
            double resprate = bioRespirationRate(cv_image, nose);
            double heartrate = bioHeartRate(cv_image, forehead);

            phyData[0] = temperature;
            phyData[1] = resprate;
            phyData[2] = heartrate;

        }
    }
    //cv::imshow("OpenCV debug", binary_image_);
    //cv::waitKey(3);
  }
  
  // Show FPS.
  if(fps_size_ == 11) {
    double fps = 11.0 / (fps_times_[10]-fps_times_[0]);
    std::ostringstream ss;
    ss << round(fps*10)/10.0;
    putText(cv_image, ss.str()+"fps", cv::Point(cv_image.cols-80, cv_image.rows-12), 3, 0.6, cv::Scalar(255, 255, 255));
  }

  return &phyData[0];
}

std::string getLevels_Hysteresis(double data, double* thresholds, double hysteresis_range, std::string method="normal") {

    std::string data_level="none";
    double thr_range = abs(thresholds[0]-thresholds[1]);
    double threshold_hysteresis[4];
    if (method.compare("normal")) {
        threshold_hysteresis[0] = thresholds[0]-thr_range*hysteresis_range;
        threshold_hysteresis[1] = thresholds[0]+thr_range*hysteresis_range;
        threshold_hysteresis[2] = thresholds[1]-thr_range*hysteresis_range;
        threshold_hysteresis[3] = thresholds[1]+thr_range*hysteresis_range;
    } else if (method.compare("upper")) {
        threshold_hysteresis[0] = thresholds[0]-thr_range*hysteresis_range;
        threshold_hysteresis[1] = thresholds[0];
        threshold_hysteresis[2] = thresholds[1];
        threshold_hysteresis[3] = thresholds[1]+thr_range*hysteresis_range;
    } else if (method.compare("lower")) {
        threshold_hysteresis[0] = thresholds[0];
        threshold_hysteresis[1] = thresholds[0]+thr_range*hysteresis_range;
        threshold_hysteresis[2] = thresholds[1]-thr_range*hysteresis_range;
        threshold_hysteresis[3] = thresholds[1];
    } else {
        ROS_INFO("Unknown hysteresis thresholding type!");
    }
    int low=0;
    int high=0;

    if (data< threshold_hysteresis[0]) {
        data_level = "low";
        low=1;
    } else if (data> threshold_hysteresis[4]) {
        data_level = "high";
        high=1;
    } else if (low==1 && (data>threshold_hysteresis[1])) {
        data_level = "normal";
        low=0;
    } else if (high==1 && (data<threshold_hysteresis[3])) {
        data_level = "normal";
        high=0;
    } else {
        /*if (i==1)
            data_level(i) = 1;
        else
            data_level(i) = data_level(i-1);        */
        data_level = "normal";
    }

    
}

void thermalImageCallback(const sensor_msgs::ImageConstPtr& raw_image) {
  //if(biometrics_pub_.getNumSubscribers() == 0)
  //  return;
  
  sensor_msgs::Image color_image;
  /*** raw (temperature) image -> RGB color image ***/
  colorConvert(raw_image, color_image);
  
  /*** ros image to opencv image ***/
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  /*** temperature decoding ***/
  unsigned short* data = (unsigned short*)&raw_image->data[0];
  for(int i = 0; i < raw_image->height; i++) {
    for(int j = 0; j < raw_image->width; j++) {
      temperature_map_[i][j] = (double(data[i*raw_image->width+j]) - 1000.0f) / 10.0f;
    }
  }
  if(data_visualization_) {
    std::fstream fs;
    fs.open("temperature", std::fstream::out | std::fstream::trunc);
    for(int i = 0; i < cv_ptr->image.rows; i++)
      for(int j = 0; j < cv_ptr->image.cols; j++)
	fs << i << " " << j << " " << temperature_map_[i][j] << "\n";
    fs.close();
  }
  physiological_monitoring::PhysiologicalData phydata;
  phydata.header = color_image.header;
  /*** face detection ***/
  double* phyParam = faceDetection(cv_ptr->image);  

  phydata.data.push_back(phyParam[0]);
  phydata.data.push_back(phyParam[1]);
  phydata.data.push_back(phyParam[2]);

  pub_phy.publish(phydata);

  // Parameter Levels
  st_anomaly_detector::StringArrayStamped phydatalevel;
  phydatalevel.header = color_image.header;
  double temp_thr[2] = {temperature_low_thr_,temperature_high_thr_};
  std::string tempLevel = getLevels_Hysteresis(phyParam[0],&temp_thr[0],1/6,"upper");
  double resp_thr[2] = {respiration_low_thr_,respiration_high_thr_};
  std::string respLevel = getLevels_Hysteresis(phyParam[1],&resp_thr[0],1/6,"upper");
  double heart_thr[2] = {heart_low_thr_,heart_high_thr_};
  std::string heartLevel = getLevels_Hysteresis(phyParam[2],&heart_thr[0],1/6,"upper");

  phydatalevel.data.push_back(tempLevel);
  phydatalevel.data.push_back(respLevel);
  phydatalevel.data.push_back(heartLevel);
  pub_phy_level.publish(phydatalevel);
  
  biometrics_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "physiological_monitoring");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  int coloring_palette;
  private_nh.param<int>("coloring_palette", coloring_palette, 6);
  image_builder_.setPalette((evo::EnumOptrisColoringPalette)coloring_palette);
  image_builder_.setPaletteScalingMethod(evo::eMinMax); // auto scaling
  
  // Default: Optris PI-450 output image size.
  int image_height, image_width;
  private_nh.param<int>("image_height", image_height, 288);
  private_nh.param<int>("image_width", image_width, 382);
  temperature_map_ = new double*[image_height];
  for(int i = 0; i < image_height; i++)
    temperature_map_[i] = new double[image_width];

  std::string ns = ros::this_node::getName();
  ns += "/";
  nh.param(ns+"temp_thr_min", temperature_min_, double(30.0));
  nh.param(ns+"temp_thr_max", temperature_max_, double(40.0));
  
  nh.param(ns+"respiration_rate_min", respiration_rate_min_, double(10.0));
  nh.param(ns+"respiration_rate_max", respiration_rate_max_, double(40.0));
  nh.param(ns+"resp_buffer_min", respiration_buffer_min_, int(100));
  nh.param(ns+"resp_buffer_max", respiration_buffer_max_, int(1000));
  respiration_buffer_ = new double[respiration_buffer_max_];
  respiration_times_ = new double[respiration_buffer_max_];

  nh.param(ns+"heart_rate_min", heart_rate_min_, double(50.0));
  nh.param(ns+"heart_rate_max", heart_rate_max_, double(180.0));
  nh.param(ns+"heart_buffer_min", heart_buffer_min_, int(10));
  nh.param(ns+"heart_buffer_max", heart_buffer_max_, int(250));
  heart_buffer_ = new double[heart_buffer_max_];
  heart_times_ = new double[heart_buffer_max_];

  nh.param(ns+"contour_area_min", contour_area_min_, int(30));

  nh.param(ns+"temp_low_thr", temperature_low_thr_, double(36.5));
  nh.param(ns+"temp_high_thr", temperature_high_thr_, double(37.3));  
  nh.param(ns+"resp_low_thr", respiration_low_thr_, double(12.0));
  nh.param(ns+"resp_high_thr", respiration_high_thr_, double(20.0));
  nh.param(ns+"heart_low_thr", heart_low_thr_, double(60.0));
  nh.param(ns+"heart_high_thr", heart_high_thr_, double(100.0));
  
  private_nh.param<bool>("data_visualization", data_visualization_, false);


  
  start_time_ = clock();
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber thermal_image_sub = it.subscribe("thermal_image", 100, thermalImageCallback); // raw image
  biometrics_pub_ = it.advertise("enrichme_biometrics", 1);

  private_nh.param("physiological_data", pub_topic_phy, std::string("/physiologicalData"));
  pub_phy = nh.advertise<physiological_monitoring::PhysiologicalData>((const std::string) pub_topic_phy, 10);

  private_nh.param("physiological_data_level", pub_topic_phylevel, std::string("/physiologicalDataLevel"));
  pub_phy_level = nh.advertise<st_anomaly_detector::StringArrayStamped>((const std::string) pub_topic_phylevel, 10);
  
  ros::spin();
  
  // Release storage space.
  if(thermal_buffer_)
    delete [] thermal_buffer_;
  
  for(int i = 0; i < image_height; i++)
    delete [] temperature_map_[i];
  delete [] temperature_map_;
  
  delete [] respiration_buffer_;
  delete [] respiration_times_;
  
  delete [] heart_buffer_;
  delete [] heart_times_;
  
  return 0;
}
