#include <tld_utils.h>
//using namespace cv;
using namespace std;

void drawBox(cv::Mat& image, CvRect box, cv::Scalar color, int thick){
  rectangle( image, cvPoint(box.x, box.y), cvPoint(box.x+box.width,box.y+box.height),color, thick);
}

void drawPoints(cv::Mat& image, vector<cv::Point2f> points,cv::Scalar color){
  for( vector<cv::Point2f>::const_iterator i = points.begin(), ie = points.end(); i != ie; ++i )
      {
      cv::Point center( cvRound(i->x ), cvRound(i->y));
      cv::circle(image,*i,2,color,1);
      }
}

cv::Mat createMask(const cv::Mat& image, CvRect box){
  cv::Mat mask = cv::Mat::zeros(image.rows,image.cols,CV_8U);
  drawBox(mask,box,cv::Scalar::all(255),CV_FILLED);
  return mask;
}

float median(vector<float> v)
{
    int n = floor(v.size() / 2);
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

vector<int> index_shuffle(int begin,int end){
  vector<int> indexes(end-begin);
  for (int i=begin;i<end;i++){
    indexes[i]=i;
  }
  random_shuffle(indexes.begin(),indexes.end());
  return indexes;
}

