
#include <tracking.h>
#include <TLD.h>
#include <tld_utils.h>


using namespace std;

Tracking::Tracking()
{
}



void Tracking::Tracking1(const cv::Rect& obj,cv::Mat  curr_image)
{
    box.x=obj.x;
    box.y=obj.y;
    box.width=obj.width;
    box.height=obj.height;
    cv::Mat frame;
    TLD  mm;
    tld=mm;
    curr_image.copyTo(frame);
    cvtColor(frame, last_gray, CV_RGB2GRAY);
    status=true;

    tld.read(1);
    bb_file = fopen("bounding_boxes.txt","w");
    tld.init(last_gray,box,bb_file);
    tl=true;

}

bool Tracking::trackobj(cv::Mat curr_image)
{


    cv::Mat frame;
    //ros::Publisher givedata=n.advertise<follower::num>("tld_people_position_estimation",1000);
    //follower::num data;
    //bb_file = fopen("bounding_boxes.txt","w");
    //int frames = 1;
    //int detections = 1;
        //get frame
    curr_image.copyTo(frame);
    cvtColor(frame, current_gray, CV_RGB2GRAY);
    tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
    if (status)
    {
            drawPoints(frame,pts1);
            drawPoints(frame,pts2,cv::Scalar(0,255,0));
            drawBox(frame,pbox);



    }
        //Display
        cv::imshow("TLD", frame);
        cv::waitKey(3);
        //swap points and images
        swap(last_gray,current_gray);
        pts1.clear();
        pts2.clear();
    if(status)
        return true;
    else
        return false;

    //fclose(bb_file);
}
