#include<iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class ImageConverter
{
    public:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber rgb_image_sub_;
        image_transport::Subscriber depth_image_sub_;
        string rgb_image_topic;
        string depth_image_topic;
        string rgb_window;
        string depth_window;
        Mat rgb;
        Mat depth;
        bool cb_show;
        bool get_depth;

        ImageConverter() : it_(nh_) {
            ros::NodeHandle nh_p("~");
            nh_p.param("rgb_image_topic", rgb_image_topic, std::string("/camera/rgb/image_raw"));
            nh_p.param("depth_image_topic", depth_image_topic, std::string("/camera/depth/image_raw"));
            nh_p.param("cb_show", cb_show, false);
            rgb_image_sub_ = it_.subscribe(rgb_image_topic, 1, &ImageConverter::rgb_imageCb, this);
            depth_image_sub_ = it_.subscribe(depth_image_topic, 1, &ImageConverter::depth_imageCb, this);
            rgb_window = "rgb";
            depth_window = "depth";
            get_depth = true;
            //cv::namedWindow("rgb");
            //cv::namedWindow("depth");
        }

        ~ImageConverter() {
            cv::destroyWindow(rgb_window);
            cv::destroyWindow(depth_window);
        }

        void rgb_imageCb(const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                //cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
                cout << "get_rgb image" << endl;
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("rgb cv_bridge exception: %s", e.what());
                return;
            }
            if(cb_show) {
                cv::imshow(rgb_window, cv_ptr->image);
                cv::waitKey(20);
            }
            rgb = cv_ptr->image.clone();
        }

        void depth_imageCb(const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                if(get_depth) {
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                    get_depth = false;
                    cout << "get_depth image" << endl;
                }
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("depth cv_bridge exception: %s", e.what());
                return;
            }
            if(cb_show) {
                cv::imshow(depth_window, cv_ptr->image);
                cv::waitKey(20);
            }
            depth = cv_ptr->image.clone();
        }
};

void detect(Mat src) {
    vector<Vec3f> original_3D_point(src.rows * src.cols);
    for(int i=0; i<src.rows; i++) {
        for(int j=0; j<src.cols; j++) {
            float x, y, z;
            y = double(src.ptr<ushort>(i)[j]) / 1000.0;
            z = (i - 314.5) * y / 570.3422;
            x = (j - 235.5) * y / 570.3422;
            original_3D_point[i*src.cols + j] = Vec3f(x, y, z);
        }
    }

    vector<Vec3f> tan_vec_x(src.rows * src.cols);
    vector<Vec3f> tan_vec_y(src.rows * src.cols);
    for (int i = 2; i<src.rows - 2; i++)
    {
        for (int j = 2; j<src.cols - 2; j++)
        {
            tan_vec_x[i*src.cols + j] = (original_3D_point[i*src.cols + j + 2] - original_3D_point[i*src.cols + j - 2]) / 2;
            tan_vec_y[i*src.cols + j] = (original_3D_point[(i+2)*src.cols + j] - original_3D_point[(i-2)*src.cols + j]) / 2;
        }
    }
    for (int i = 0; i<src.cols; i++)
    {
        tan_vec_x[i] = Vec3f(0, 0, 0);
        tan_vec_x[i + src.cols] = Vec3f(0, 0, 0);
        tan_vec_x[src.cols*(src.rows - 1) + i] = Vec3f(0, 0, 0);
        tan_vec_x[src.cols*(src.rows - 2) + i] = Vec3f(0, 0, 0);
    }
    for (int j = 1; j<src.rows - 1; j++)
    {
        tan_vec_y[j*src.cols - 1] = Vec3f(0, 0, 0);
        tan_vec_x[j*src.cols - 2] = Vec3f(0, 0, 0);
        tan_vec_y[j*src.cols] = Vec3f(0, 0, 0);
        tan_vec_x[j*src.cols + 1] = Vec3f(0, 0, 0);
    }

    //计算法线
    vector<Vec3f> nor_vec_n(src.cols*src.rows);
    float tempx, tempy, tempz, pos;
    for(int i=0; i<src.rows; i++) {
        for(int j=0; j<src.cols; j++) {
            pos = i * src.cols + j;
            tempx = tan_vec_x[pos](1)*tan_vec_y[pos](2) - tan_vec_x[pos](2)*tan_vec_y[pos](1);
            tempy = tan_vec_x[pos](2)*tan_vec_y[pos](0) - tan_vec_x[pos](0)*tan_vec_y[pos](2);
            tempz = tan_vec_x[pos](0)*tan_vec_y[pos](1) - tan_vec_x[pos](1)*tan_vec_y[pos](0);
            nor_vec_n[pos] = Vec3f(tempx, tempy, tempz);
        }
    }

    //归一化
    float mold;
    for (int i=0; i<src.rows; i++) {
        for (int j=0; j<src.cols; j++) {
            pos = i * src.cols + j;
            mold = sqrt(pow(nor_vec_n[pos](0), 2) + pow(nor_vec_n[pos](1), 2) + pow(nor_vec_n[pos](2), 2));
            nor_vec_n[pos] = Vec3f(nor_vec_n[pos](0) / mold, nor_vec_n[pos](1) / mold, nor_vec_n[pos](2) / mold);
            //cout<<mold<<"  "<<nor_vec_n.at(pos)[0]<<"  "<<nor_vec_n.at(pos)[1]<<"  "<<nor_vec_n.at(pos)[2]<<endl;
        }
    }

    //image segmentation and clustering
    //threshold1,threshold2
    vector<int> temp1;
    vector<int> temp2;
    vector<float> z_buff;
    vector<int>::iterator iter;
    ////////////////////////////////////////////////////////////////////////////////////////////
    float threshold1 = 0.75;//0.85;
    ////////////////////////////////////////////////////////////////////////////////////////////
    int length = 0;
    int i = 0;
    int j = 0;
    //cout<<"start"<<endl;
    //temp1中存储哪些像素位置的法向量z方向符合初筛
    //z_buff中存储对应位置的法向量z方向数值等待聚类处理
    for (i = 0; i<src.cols*src.rows; i++) {
        if (abs(nor_vec_n[i](2)) > threshold1) {
            temp1.push_back(i);
            temp2.push_back(0);//是不是有必要在下一节代码中定义temp2
            //j = original_3D_point[i](2);
            //z_buff.push_back(j);
            z_buff.push_back(original_3D_point[i](2));
        }
    }
    i = 0;
    j = 0;
    Mat image1 = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    //Mat image1(src.rows, src.cols, CV_8UC1);//������ʾ����z����������һ����ֵ�ĵ�
    Mat image2(src.rows, src.cols, CV_8UC1);//������ʾ�ָ���
    Mat image3 = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);//������ʾ�ָ���
    //CvMat *image3=cvCreateMat(DEPTH_HEIGHT,DEPTH_WIDTH,CV_8UC3);//������ʾ�ָ���
    for (iter = temp1.begin(); iter != temp1.end(); iter++)
    {
        i = (*iter) % src.cols;
        j = (*iter) / src.cols;
        image1.at<uchar>(j, i) = 255;
    }
    //namedWindow("image1");//显示所有符合初筛的法向量的位置
    imshow("image1",image1);
    waitKey(30);

    length = temp1.size();
    //vector<int> leibie;
    vector<int> shuliang;
    vector<float> junzhi;
    ////////////////////////////////////////////////////////////////////////////////////////////
    float threshold2 = 0.03;//55;
    ////////////////////////////////////////////////////////////////////////////////////////////
    int add_flag;
    //leibie.push_back(0);//????

    shuliang.push_back(1);
    junzhi.push_back(z_buff[0]);
    temp2[0] = 1;


    for (i = 1; i<length; i++)
    {
        add_flag = 1;
        //cout<<z_buff[i]<<"  ";
        for (j = 0; j<shuliang.size(); j++)
        {
            //cout << z_buff[i] << endl;
            if (abs(z_buff[i] - junzhi[j]) < threshold2)//print z_buff num!!!!!!!!
            {
                shuliang[j] = shuliang[j] + 1;
                junzhi[j] = (junzhi[j] * (shuliang[j] - 1) + z_buff[i]) / shuliang[j];
                temp2[i] = j + 1;
                add_flag = 0;
                break;
            }
        }
        if (add_flag)
        {
            if (j == 107)
            {
                cout << "i am here" << endl;
            }
            temp2[i] = j + 1;
            shuliang.push_back(1);
            junzhi.push_back(z_buff[i]);
        }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////
    int threshold3 = 10000;//8000;
    ////////////////////////////////////////////////////////////////////////////////////////////
    //cout<<"hello  "<<shuliang.size()<<endl;
    //cout<<"length "<<length<<endl;
    //for (iter=shuliang.begin();iter!=shuliang.end();iter++)
    for (j = 0; j<shuliang.size(); j++)
    {
        if (shuliang[j] < threshold3)
        {
            for (i = 0; i<length; i++)
            {
                if (temp2[i] == (j + 1))
                {
                    temp2[i] = 0;
                }
            }
        }
        else
        {
            cout << j << "   " << shuliang[j] << endl;
        }
    }
    //for (iter=temp1.begin();iter!=temp1.end();iter++)
    //{
    //	i=(*iter)%DEPTH_WIDTH;
    //	j=(*iter)/DEPTH_WIDTH;
    //	image2.at<uchar>(j,i)=255;
    //}
    //cvSetZero(image3);
    for (i = 0; i<length; i++)
    {
        //image2.at<uchar>(temp1[i]/DEPTH_WIDTH,temp1[i]%DEPTH_WIDTH)=255;
        if (temp2[i])
        {
            //cout<<temp2[i]<<" ";//<<i<<endl;
            image2.at<uchar>(temp1[i] / src.cols, temp1[i] % src.cols) = (temp2[i] * 17) % 256;
            //image3[temp1[i]/DEPTH_WIDTH][temp1[i]%DEPTH_WIDTH]=Vec3b((i*30%200)+55,(i*50%200)+55,(i*90%200)+40);
            image3.at<Vec3b>(temp1[i] / src.cols, temp1[i] % src.cols) = Vec3b(0, 255, 0);//Vec3b((temp2[i] * 20 % 200) + 55, (temp2[i] * 50 % 200) + 55, (temp2[i] * 100 % 200) + 40);
        }
    }
    //namedWindow("image2");
    //imshow("image2",image2);
    namedWindow("image3");
    imshow("image3", image3);
    //cvnamedWindow("image3");
    //imshow("image3",image3);

}

int main(int argc, char** argv)
{
    cout << "test" << endl;
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    Mat test,test1;
    ros::Rate loop_rate(30);
    
    while (ros::ok()) {
        test = ic.rgb;
        test1 = ic.depth;
        if(test.data) {
            imshow("test", test);
            waitKey(3);
        }
        if(test1.data) {
            detect(test1);
            imshow("test1", test1);
            ic.get_depth = true;
            cout << "reset flag" << endl;
            waitKey(3);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }/*
    while(1) {
        if(!ic.get_rgb && !ic.get_depth) {
            ic.get_depth = true;
            ic.get_rgb = true;
            test = ic.rgb;
            test1 = ic.depth;
            if(test.data) {
                imshow("test", test);
                waitKey(3);
            }
            if(test1.data) {
                detect(test1);
                imshow("test1", test1);
                waitKey(3);
            }
        }

    }*/
    return 0;
}
