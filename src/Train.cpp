

#include "Train.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>

//OpenCV specific includes
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <opencv2/flann/flann.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <cstdlib>
//using namespace std;
//using namespace cv;


Train::Train()
{


}
Train::~Train()
{

}

int Train::train(const int& sum)
{
     //cv::initModule_nonfree();
     /*vector <Mat> trainImages1;
    vector<vector<KeyPoint> > trainKeyPoints1;
    vector<Mat> trainDescriptors1;*/
    std::vector<std::vector<cv::KeyPoint> >  objectclassKeypoint[31];    //最多30个物品
    std::vector<cv::Mat>  objectclassDescriptors[31];                        //最多30个物品
    std::vector <cv::Mat> objectclassimages[31];
    std::vector <cv::Mat> objectclassimages_bgr[31];

    std::vector<std::string>  trainFilenames;
    std::vector<cv::KeyPoint> middle;
    cv::Mat img;
    cv::Mat img1;
    cv::Mat des;
     std::string str;

     std::stringstream ss;
     cv::FileStorage fs("/home/ubuntu/object_recognition/image.xml", cv::FileStorage::WRITE);
    cv::FileStorage fs1("/home/ubuntu/object_recognition/object.xml", cv::FileStorage::WRITE);
  
    double width[30];
    std::string obj_name[30];
    std::string filename;


    std::ifstream file("/home/ubuntu/object_recognition/name.txt");
    std::ifstream file1("/home/ubuntu/object_recognition/width.txt");
    std::ifstream file2("/home/ubuntu/object_recognition/obj_name.txt");
    fs<<"sum"<<sum;


    if(!file.is_open())
    {

        return -1;
    }
    while(!file.eof())
    {

        getline(file,str);
        if(str.empty())
            break;
        trainFilenames.push_back(str);

    }
    if(!file1.is_open())
    {

        return -1;
    }
    int cixu=1;
    while(!file1.eof())
    {

        file1>>width[cixu];
        cixu++;

    }
      if(!file2.is_open())
    {

        return -1;
    }
    cixu=1;
    while(!file2.eof())
    {

        file2>>obj_name[cixu];
        cixu++;

    }
    file.close();
    file1.close();
    file2.close();
     int minHessian = 400;
    cv::SurfFeatureDetector  featureDetector( minHessian );
    cv::Ptr<cv::DescriptorExtractor>  descriptorExtractor=cv::DescriptorExtractor::create( "SURF" );

    //cv::SurfDescriptorExtractor descriptorExtractor;
     //FlannBasedMatcher descriptorMatcher ;
    //featureDetector = FeatureDetector::create( "SURF" );
    //Ptr<DescriptorExtractor>  descriptorExtractor = DescriptorExtractor::create(  "SURF" );
    //descriptorMatcher = DescriptorMatcher::create( "FlannBased" );
     //bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );
    //if( !isCreated )
        //cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;
    int num;
    for( size_t i = 0; i <  trainFilenames.size(); i++ )
    {
        middle.clear();
        filename = trainFilenames[i];
        img = cv::imread( "/home/ubuntu/object_recognition/"+filename+".jpg", CV_LOAD_IMAGE_GRAYSCALE );
        img1=cv::imread( "/home/ubuntu/object_recognition/"+filename+".jpg" );
        if( img.empty() )
        {
            std::cout << "Train image " << filename << " can not be read." << std::endl;
            return 0;
        }
        else
        {//readImageCount++;
      //  featureDetector.detect( img, middle );
        //vector<KeyPoint>::iterator iter;
        std::string cui;
        int kkk;
        for(kkk=0;kkk<filename.size();kkk++)
            {if(filename[kkk]!='_')
               cui+=filename[kkk];
            else
                break;
            }
       num=atoi(cui.c_str());

       // for (iter=middle.begin();iter!=middle.end();iter++)
      ////  {
          //  (*iter).class_id=num;
        //}
       // trainKeyPoints.push_back(middle);
        //objectclassKeypoint[num].push_back(middle);
        //featureDetector.detect(img, middle );
        //descriptorExtractor->compute( img, middle, des);
        //fs<<filename;
        //fs<<"{"<<"KeyPoints"<<middle;
        //fs<<"Descriptors"<<des<<"}";
        objectclassimages[num].push_back(img);
        objectclassimages_bgr[num].push_back(img1);
        cui.clear();

       // trainImages.push_back( img );}
        }
    }

    //featureDetector.detect(trainImages, trainKeyPoints );
    //descriptorExtractor.compute( trainImages, trainKeyPoints, trainDescriptors);
    int i;
    for (i=1;i<=sum;i++)
    {
            featureDetector.detect(objectclassimages[i],objectclassKeypoint[i]);
            descriptorExtractor->compute(objectclassimages[i],objectclassKeypoint[i],objectclassDescriptors[i]);
    }
     //std::cout<<"请按编号输入每个物品的名字和宽度"<<std::endl;
    int iii,jjj;
    fs1<<"sum"<<sum;
    for(iii=1;iii<=sum;iii++)
    {
        /*std::cout<<"第"<<iii<<"个物品的名字"<<std::endl;
        std::cin>>name[iii];
         std::cout<<"第"<<iii<<"个物品的宽度"<<std::endl;
        std::cin>>width[iii];*/
        //ss<<iii;
        fs1<<obj_name[iii];
        fs1<<"{";
        //fs1<<"name"<<name[iii];
        fs1<<"Image";//<<"[";
        //for(jjj=0;jjj<objectclassimages[iii].size();jjj++)
        {
            fs1<<objectclassimages_bgr[iii];
        }
        //fs1<<"]";
        fs1<<"Descriptor";//<<"[";
        //or(jjj=0;jjj<objectclassimages[iii].size();jjj++)
        {
            fs1<<objectclassDescriptors[iii];
        }
        //fs1<<"]"<<"}";
        fs1<<"}";
        //ss.clear();
    }
    fs1<<"name"<<"[";
    for(iii=1;iii<=sum;iii++)
    {
        fs1<<obj_name[iii];
    }
    fs1<<"]";
    fs1<<"width"<<"[";
    for(iii=1;iii<=sum;iii++)
    {
        fs1<<width[iii];
    }
    fs1<<"]";
   /* trainImages= trainImages1;
    trainKeyPoints=trainKeyPoints1;
    trainDescriptors=trainDescriptors1;
    objectclassKeypoint=objectclassKeypoint1;    //最多30个物品
    objectclassDescriptors =objectclassDescriptors1;                        //最多30个物品
    objectclassimages=objectclassimages;*/
    fs.release();
    fs1.release();
    return 1;



}

