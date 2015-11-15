//从文件中获取图片名称，载入，分批进行处理，最后识别
// OpenCV specific includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <setjmp.h>
#include <tracking.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <xm_object_cui/num.h>
#include <xm_msgs/xm_Object.h>
#include <xm_msgs/xm_ObjectDetect.h>
#include <xm_TLD/pubdata.h>
#include <hpdf.h>
#include "rosgetimage.h"
#include "Filter.h"
#include "detect.h"


cv::Rect box;
std::string  object_name;
int flag=0;
int sum1;
int num=0;
int num1=0;
string name[31];
double width[31];
bool istrue;
cv::Mat image;
bool yess=false;
//Tracking gen;
time_t timep;
Filter filter;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudP;
enum Process_State {free1,recognition};
Process_State State;
//ros::Subscriber sub;
std::vector<cv::Mat>  descriptor[31];
std::vector<cv::Mat> image_obj[31];
std::vector <cv::MatND> hist[31];
Detection detect;
pcl::visualization::CloudViewer viewer("X");
float center_x;
float center_y;
float center_z;
float ux;
float uy;
std::vector<Filter::Box2DPoint> my_points;
//ros::Publisher givedata;

jmp_buf env;
void error_handler  (HPDF_STATUS   error_no,
                HPDF_STATUS   detail_no,
                void         *user_data)
{
    printf ("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
                (HPDF_UINT)detail_no);
    longjmp(env, 1);
}

HPDF_Doc  pdf;
HPDF_Font font;
HPDF_Page page;
//*** This path specify location where the file of object recognition's result,  
// you can replace it to what you like ***
string fname="/home/ubuntu/xmbot_object_recognition.pdf";
HPDF_Destination dst;


bool result(xm_msgs::xm_ObjectDetect::Request  &req,
         xm_msgs::xm_ObjectDetect::Response &res)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_RGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *cloud_RGBA=*cloudP;
    my_points=filter.Cluster(cloud_RGBA);
    cout<<my_points.size()<<endl;
    if(my_points.size()==0)
    {
        cout<<"no points"<<endl;
        return false;
    }
    //cloud_RGBA=filter.RemovePlane(cloud_RGBA);
    num=req.object_id;
    cout<<"yes1"<<endl;

    res.object.name.data=name[num];
    res.object.id=num;
    yess=detect.detect_object(image,my_points,num,descriptor[num],hist[num]);
    if(yess==false)
        return false;
    ux=detect.track_object.x+detect.track_object.width/2.0;
    uy=detect.track_object.y+detect.track_object.height/2.0;
    center_z=(float)((cloudP->at(ux, uy)).z);

    center_x=(ux-320)*(center_z)/525.0;
	center_y=(uy-240)*(center_z)/525.0;
	res.object.pos.y=-center_x-0.045;
    res.object.pos.z=-center_y;
    res.object.pos.x=center_z;
    res.object.width=width[num];
    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "xm_object_cui");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("Find_Object", result);
    ros::Rate waiting_rate(30);
    cv::initModule_nonfree();

    if(flag==0)
    {
        //*** This path specify location where the file of object's informations, 
        // you need replace ros workspace to what you are using ***
        cv::FileStorage fs("/home/ubuntu/Desktop/ros_ws/xm_test_ws/src/xm_object_cui/config/object_recognition/object.xml", cv::FileStorage::READ);
        sum1=(int)fs["sum"];
        int i;
        cv::FileNode name_obj = fs["name"];
        cv::FileNodeIterator  it_obj = name_obj.begin(), it_obj_end = name_obj.end();
        for(i=1;it_obj!=it_obj_end;++it_obj,++i)
        {name[i]=(string)(*it_obj);

        }
        cv::FileNode width_obj = fs["width"];
        cv::FileNodeIterator  width_it_obj = width_obj.begin(), width_it_obj_end = width_obj.end();
        for(i=1;width_it_obj!=width_it_obj_end;++width_it_obj,++i)
        {width[i]=(double)(*width_it_obj);

        }
        for(i=1;i<=sum1;i++)
        {
            //ss<<i;
            //descriptor[i]=(std::vector<cv::Mat>)(fs[name[i]]["Descriptor"]);
            cv::FileNode descriptor1 = fs[name[i]]["Descriptor"];
            cv::FileNodeIterator  it = descriptor1.begin(), it_end = descriptor1.end();

            for(;it!=it_end;++it)
            {
                // std::vector<cv::Mat> aaa;
                it>>descriptor[i];
            }

        }
        // cout<<descriptor[1].size()<<" "<<descriptor[2].size()<<endl;
        for(i=1;i<=sum1;i++)
        {
            //ss<<i;
            //descriptor[i]=(std::vector<cv::Mat>)(fs[name[i]]["Descriptor"]);
            cv::FileNode image1 = fs[name[i]]["Image"];
            cv::FileNodeIterator  it_image = image1.begin(), it_image_end = image1.end();

            for(;it_image!=it_image_end;++it_image)
            {
                        // std::vector<cv::Mat> aaa;
                it_image>>image_obj[i];
            }
            //ss.clear();
        }
        //cout<<image_obj[1].size()<<" "<<image_obj[2].size()<<endl;
        int h_bins = 50; int s_bins = 60;
        int histSize[] = { h_bins, s_bins };

        // hue的取值范围从0到256, saturation取值范围从0到180
        float h_ranges[] = { 0, 256 };
        float s_ranges[] = { 0, 180 };

        const float* ranges[] = { h_ranges, s_ranges };

        // 使用第0和第1通道
        int channels[] = { 0, 1 };
        for(i=1;i<=sum1;i++)
        {
            int ll=image_obj[i].size();
            //std::cout<<"ll"<<ll<<std::endl;
            for(int j=0;j<ll;j++)
            {
                cv::Mat hsv_base;
                cv::MatND hist_base;
                cvtColor( image_obj[i][j], hsv_base, CV_BGR2HSV );
                cv::calcHist( &hsv_base, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
                cv::normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
                hist[i].push_back(hist_base);
            }
        }

        pdf = HPDF_New (error_handler, NULL);
        if (!pdf) {
            printf ("error: cannot create PdfDoc object\n");
            return 1;
        }

        /* error-handler */
        if (setjmp(env)) {
            HPDF_Free (pdf);
            return 1;
        }

        HPDF_SetCompressionMode (pdf, HPDF_COMP_ALL);

        /* create default-font */
        font = HPDF_GetFont (pdf, "Helvetica", NULL);


        flag=1;
    }
    rosgetimage ct(n);

     while(!ct.isReady())
     {
         ros::spinOnce();
         waiting_rate.sleep();
     }
    State=free1;
    cvNamedWindow("CurrentImage",CV_WINDOW_AUTOSIZE);

    while(ros::ok())
    {
        while(!ct.isReady())
        {
          ros::spinOnce();
        }

        image=ct.getImage();
        cloudP=ct.getCloud();

        if(cloudP->empty())
        {
            std::cout<<"No pointCloud passed into this process, fuck you no points you play MAOXIAN!"<<std::endl;
            continue;
        }

        //Detection detect;

            if((num!=num1)&&(yess==true))
            {

               // gen.Tracking1(detect.track_object,image);
                cv::rectangle(image,cv::Point(detect.track_object.x,detect.track_object.y),cv::Point(detect.track_object.x+detect.track_object.width,detect.track_object.y+detect.track_object.height),cv::Scalar(255,0,0));
                //*** This path specify location where the file of object's informations, 
                // you need replace ros workspace to what you are using ***
                string my_string="/home/ubuntu/Desktop/ros_ws/xm_test_ws/src/xm_object_cui/config/object_recognition/" + name[num]+".jpg";
                cv::imwrite(my_string,image);
                char key2 = cvWaitKey(10);
                if ( key2 == 27)
                {break;}

                page = HPDF_AddPage (pdf);

                HPDF_Page_SetWidth (page, 650);
                HPDF_Page_SetHeight (page, 500);

                dst = HPDF_Page_CreateDestination (page);
                HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page), 1);
                HPDF_SetOpenAction(pdf, dst);


                HPDF_Page_SetFontAndSize (page, font, 12);
                HPDF_Image pdf_image;
                pdf_image=HPDF_LoadJpegImageFromFile (pdf, my_string.c_str());

                HPDF_Page_DrawImage (page, pdf_image, 70, HPDF_Page_GetHeight (page) - 410,HPDF_Image_GetWidth (pdf_image),HPDF_Image_GetHeight (pdf_image));
                std::stringstream ss;

                time(&timep);
                ss<<ctime(&timep);

                HPDF_Page_BeginText (page);
                HPDF_Page_SetTextLeading (page, 16);
                HPDF_Page_MoveTextPos (page, 70, HPDF_Page_GetHeight (page) - 410);
                HPDF_Page_ShowTextNextLine (page, name[num].c_str());
                HPDF_Page_ShowTextNextLine (page, ss.str().c_str());
                HPDF_Page_EndText (page);
                HPDF_SaveToFile (pdf, fname.c_str());
                num1=num;

            }
            //else
               // istrue=gen.trackobj(image);
                    //detect.tracking();
                        //gen.Tracking1(detect.track_object,image);
           // cv::rectangle(image,cv::Point(detect.track_object.x,detect.track_object.y),cv::Point(detect.track_object.x+detect.track_object.width,detect.track_object.y+detect.track_object.height),cv::Scalar(255,0,0));
                        /*std_msgs::String massg;
                        std::stringstream ss11;
                        ss11<<detect.track_object.x<<","<<detect.track_object.y<<","<<detect.track_object.width<<","<<detect.track_object.height;
                        massg.data=ss11.str();
                        while(1)
                        {
                            client = n.serviceClient<follower::pubdata>("givedata");
                            follower::pubdata srv;
                            srv.request.a=massg.data;
                            if (client.call(srv))
                            {
                                zhao=1;
                                break;
                            }
                        }*/
            //if(istrue==true)
            //{
            //ux=gen.pbox.x+gen.pbox.width/2.0;
            //uy=gen.pbox.y+gen.pbox.height/2.0;
            /*if (!cloudP.data)
            {
                cout<<"no data!"<<endl;
                return 0;
            }*/
            //center_z=(float)((cloudP->at(ux, uy)).z);

            //center_x=(ux-320)*(center_z)/525.0;
           // center_y=(uy-240)*(center_z)/525.0;
            //data.a=num;
            //data.y=-center_x-0.045;
            //data.z=-center_y;
           // data.x=center_z;
            //cout<<data.a<<" "<<data.x<<" "<<data.y<<" "<<data.z<<endl;
            //givedata.publish(data);
            //}
            //else
                //num1=0;


        viewer.showCloud(cloudP);
        cv::imshow("CurrentImage", image);
        char temp=cvWaitKey(33);
        if(temp=='q')
        {
             HPDF_SaveToFile (pdf, fname.c_str());

                /* clean up */
            HPDF_Free (pdf);
            return 0;
        }

    }
}
















