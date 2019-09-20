//
// Created by wendy on 2019/9/10.
//

#include <opencv2/opencv.hpp>
//#include <Eigen/Core>
//#include <pangolin/pangolin.h>
//#include <g2o/core/g2o_core_api.h>
//#include <ceres/ceres.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <map>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;
namespace bf = boost::filesystem;

struct OdoData {
    unsigned long timeStamp;
    double x, y, theta;
};

void readodo(const string rootpath, vector<OdoData> &odomDatas, vector<string> &writedatas);

void readimage(bf::path path, vector<Mat> &image);

void savedata(const vector<OdoData> ododata);

void wirtedata(vector<Mat> &imagedata, vector<string> ododata);

void reimage(vector<Mat> src, vector<Mat> &dst, double size);

int main(int argc, char **argv) {

    //判断输入参数是否正确
    if (argc != 2) {
        cerr << "Wrong number of input arguments! " << endl;
        cerr << "Usage: " << argv[0] << " <Data_Path>" << endl;
        return -1;
    }

    boost::filesystem::path root_path(argv[1]);
    if (!boost::filesystem::exists(root_path)) {
        cerr << "Data folder doesn't exist!" << endl;
        return -1;
    }


    vector<OdoData> odoData;
    vector<string> writedatas;
    readodo(argv[1], odoData, writedatas); //读里程计数据
    cout << "the size of odometry data is:" << odoData.size() << endl;


    bf::path path("/home/wendy/study/myDataset/image2");
    if (!bf::exists(path)) {
        cerr << "Data folder doesn't exist!" << endl;
    }

    vector<Mat> image;
    readimage(path, image); // 读图片
    cout << "the size of image is:" << image.size() << endl;
    cout << "image.rows is : " << image[0].rows << endl << "image.cols is: " << image[0].cols << endl;

    wirtedata(image, writedatas); //将里程计数据写入图片

    vector<Mat> dstimage;
    reimage(image, dstimage, 2.0);
    for(int i=0; i<dstimage.size(); ++i){
        imshow("dstimage", dstimage[i]);
//        cout << "[wqw] test show" << endl;
        waitKey(40);
    }


    savedata(odoData); //写入文件

    return 0;
}


// 2.读里程计数据
void readodo(const string rootpath, vector<OdoData> &odomDatas, vector<string> &writedatas) {
    ifstream odofile(rootpath + "/odom_raw.txt", ios::in);
    if (!odofile) {
        cout << "error opening odom_raw file." << endl;
        exit(-1);
    }

    string line;

    while (getline(odofile, line) && !line.empty()) {
        vector<string> l;
        boost::split(l, line, boost::is_any_of(" \t"), boost::token_compress_on);
        if (l.size() == 8) {
            OdoData od;
            od.timeStamp = atol(l[0].c_str());
            od.x = atof(l[1].c_str());
            od.y = atof(l[2].c_str());
            od.theta = atof(l[3].c_str());
            odomDatas.push_back(od);
            writedatas.push_back(l[7]);

        }
    }
    odofile.close();

    if (odomDatas.empty())
        cerr << "No odo data in this file:" << rootpath << "/odom_raw.txt" << endl;
}

// 3.读图片
void readimage(bf::path path, vector<Mat> &image) {
    bf::directory_iterator end_iter;
    string imageline;
    map<int, string> timemap;

    for (bf::directory_iterator iter(path); iter != end_iter; ++iter) {
        if (bf::is_directory(iter->status()))
            continue;
        if (bf::is_regular_file(iter->status())) {
            // format: /frameRaw981078195.jpg
            imageline = iter->path().string();
            if (!boost::algorithm::ends_with(imageline, "jpg"))
                continue;
            auto i = imageline.find_last_of('w');
            auto j = imageline.find_last_of('.');
            long long int timestamp = atoll(imageline.substr(i + 1, j - i - 1).c_str()); //atoll(将char×转变为longlongint）
            timemap.insert(pair<int, string>(timestamp, imageline));
        }
    }
    map<int, string>::iterator iter;
    for (iter = timemap.begin(); iter != timemap.end(); ++iter) {
        image.push_back(imread(iter->second));
    }
}

//TODO 4.选出里程计数据中的前m个数据, 将其中数据的最后一个值保留2位小数, 写在对应的图像左上角合适的位置;
void wirtedata(vector<Mat> &imagedata, vector<string> ododata) {
    char dt[10];

    for (int i = 0; i < imagedata.size(); ++i) {
        sprintf(dt, "%.2f", ododata[i].c_str());
        string strTheta = "d_theta: " + string(dt);
        cv::putText(imagedata[i], strTheta, Point(180, 80), 1, 1.1, Scalar(0, 255, 0), 1);
    }

}

// 5.缩放图片
void reimage(vector<Mat> src, vector<Mat> &dst, double size) {

    for (int i = 0; i < src.size(); ++i){
        Mat dstimage;
        cv::resize(src[i], dstimage, Size(), size, size);
        dst.push_back(dstimage);
    }


}

// 6.写出文件
void savedata(const vector<OdoData> ododata) {

    ofstream save_data;
    save_data.open("../odom_out.txt");

    for (int i = 0; i < ododata.size(); ++i) {
        save_data << "x: " << ododata[i].x << "    y: " << ododata[i].y << "    theta: " << ododata[i].theta << endl;
    }
    save_data.close();

}