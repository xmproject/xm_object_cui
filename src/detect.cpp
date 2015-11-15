//只要寻找要识别的物体在哪里就可以了。。。。

#include "detect.h"
#include <algorithm>
using namespace std;
//using namespace cv;

/*void Detection::tracking()
{

}*/


bool Detection::detect_object(cv::Mat image,const std::vector<Filter::Box2DPoint>& my_points_1,const int& s,const vector<cv::Mat>&  des,const vector<cv::MatND>& hist)
{

    int num;
    int minHessian = 400;
    std::vector<Filter::Box2DPoint> my_points;
    cv::SurfFeatureDetector  featureDetector( minHessian );
    cv::SurfDescriptorExtractor descriptorExtractor;
    cv::FlannBasedMatcher  descriptorMatcher;

    cv::Mat queryImage[30];
    cv::Mat queryDescriptors[30];
    vector<cv::KeyPoint> queryKeypoints[30];
    cv::Rect box;

    int length=my_points_1.size();
   //std::cout<<length<<std::endl;
   int wen=0;
    for(int k=0;k<length;)
    {
        cv::Mat frame;
        if((my_points_1[k].x_left_down<0)||(my_points_1[k].y_left_down<0)||(my_points_1[k].x_right_up>image.cols)||(my_points_1[k].y_right_up>image.rows))
        {
            k++;
            continue;
        }
        if(((my_points_1[k].x_right_up-my_points_1[k].x_left_down)>300)||((my_points_1[k].y_right_up-my_points_1[k].y_left_down)>350))
        {
            k++;
            continue;
        }
        if(((my_points_1[k].x_right_up-my_points_1[k].x_left_down)<0)||((my_points_1[k].y_right_up-my_points_1[k].y_left_down)<0))
        {
            k++;
            continue;
        }
        box.x=my_points_1[k].x_left_down;
        box.y=my_points_1[k].y_left_down;
        box.width=my_points_1[k].x_right_up-my_points_1[k].x_left_down;
        box.height=my_points_1[k].y_right_up-my_points_1[k].y_left_down;

        my_points.push_back(my_points_1[k]);
         cout<<"yes2"<<endl;
        image(box).copyTo(queryImage[wen]);
        cout<<box.x<<" "<<box.y<<" "<<box.width<<" "<<box.height<<endl;
        char key = cvWaitKey(10);
        if ( key == 27)
            {break;}
        //queryImage[k]=Mat(image,box).clone();

        cv::cvtColor(queryImage[wen],frame,CV_BGR2GRAY);
        featureDetector.detect( frame, queryKeypoints[wen] );
        descriptorExtractor.compute( frame, queryKeypoints[wen], queryDescriptors[wen] );
        wen++;
        k++;
    }

    length=wen;
    cout<<length<<endl;
    if(s!=1000)
    {

        descriptorMatcher.add(des);
	cout<<des.size()<<endl;
        descriptorMatcher.train();
        vector<cv::DMatch> matches[30];
         vector<cv::DMatch>   jjj;
        vector< cv::DMatch > good_matches[30];
        int k;

        for(k=0;k<length;k++)
        {
            descriptorMatcher.match( queryDescriptors[k], jjj);
            matches[k]=jjj;
            //std::cout<<matches[k].size()<<std::endl;

            //std::cout<<min_dist<<std::endl;
            for( int i = 0; i < queryDescriptors[k].rows; i++ )
            {
                if( matches[k][i].distance < 0.3 )
                    { good_matches[k].push_back( matches[k][i]); }
            }

        }
        //std::cout<<good_matches[0].size()<<"  "<<good_matches[1].size()<<std::endl;
        cout<<"yes3"<<endl;
        Point point[30];
        for (int i=0;i<length;i++)
        {
            point[i].x=i;
            point[i].y=good_matches[i].size();
        }
        sort(point,point+length,Detection::compare());
        //cout<<point[0].x<<" "<<point[0].y<<" "<<point[1].x<<" "<<point[1].y<<" "<<point[2].x<<" "<<point[2].y<<endl;
        if(length==1)
        {
            track_object.x=my_points[0].x_left_down;
            track_object.y=my_points[0].y_left_down;
            track_object.width=my_points[0].x_right_up-my_points[0].x_left_down;
            track_object.height=my_points[0].y_right_up-my_points[0].y_left_down;
            return true;
        }


        else if(((point[1].y==0)&&(point[0].y>5)))
        {
            track_object.x=my_points[point[0].x].x_left_down;
            track_object.y=my_points[point[0].x].y_left_down;
            track_object.width=my_points[point[0].x].x_right_up-my_points[point[0].x].x_left_down;
            track_object.height=my_points[point[0].x].y_right_up-my_points[point[0].x].y_left_down;
            return true;
        }
        else if(point[1].y!=0&&(point[0].y/point[1].y>=3)&&(point[0].y>5))
        {
            track_object.x=my_points[point[0].x].x_left_down;
            track_object.y=my_points[point[0].x].y_left_down;
            track_object.width=my_points[point[0].x].x_right_up-my_points[point[0].x].x_left_down;
            track_object.height=my_points[point[0].x].y_right_up-my_points[point[0].x].y_left_down;
            return true;
        }
        else
        {
            int imagenum=hist.size();
            double hismax[2];
            hismax[0]=-100.0;
            hismax[1]=-100.0;
            cv::Mat hsv_base;
            cv::MatND hist_base;
            int h_bins = 50; int s_bins = 60;
            int histSize[] = { h_bins, s_bins };

            // hue的取值范围从0到256, saturation取值范围从0到180
            float h_ranges[] = { 0, 256 };
            float s_ranges[] = { 0, 180 };

            const float* ranges[] = { h_ranges, s_ranges };

                    // 使用第0和第1通道
            int channels[] = { 0, 1 };
            for(int kk=0;kk<2;kk++)
            {
                cv::cvtColor( queryImage[point[kk].x], hsv_base, CV_BGR2HSV );
                cv::calcHist( &hsv_base, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
                cv::normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

                for(int ii=0;ii<imagenum;ii++)
                {

                    double base_test1 = compareHist( hist_base, hist[ii], 0 );
                    if(base_test1>hismax[kk])
                        hismax[kk]=base_test1;
                }
            }
            if(hismax[0]<0.5&&hismax[1]<0.5)
                return false;
            else if(hismax[0]>hismax[1])
            {
                track_object.x=my_points[point[0].x].x_left_down;
                track_object.y=my_points[point[0].x].y_left_down;
                track_object.width=my_points[point[0].x].x_right_up-my_points[point[0].x].x_left_down;
                track_object.height=my_points[point[0].x].y_right_up-my_points[point[0].x].y_left_down;
                cout<<hismax[0]<<endl;
                return true;
            }
            else
            {
                track_object.x=my_points[point[1].x].x_left_down;
                track_object.y=my_points[point[1].x].y_left_down;
                track_object.width=my_points[point[1].x].x_right_up-my_points[point[1].x].x_left_down;
                track_object.height=my_points[point[1].x].y_right_up-my_points[point[1].x].y_left_down;
                cout<<hismax[1]<<endl;
                return true;
            }
        }


    }

}




