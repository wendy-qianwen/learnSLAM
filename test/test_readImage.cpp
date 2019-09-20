//
// Created by wendy on 2019/9/5.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;

void ReadIntrinsics(Mat &cameraMatrix, Mat &distCoeffs, Size &imageSize, const char *IntrinsicsPath);
void Undistort_img(Mat map1, Mat map2, Mat nowimage);
void mergeImg(Mat & dst, const Mat &src1, const Mat &src2);



int main() {
    std::string pattern_jpg = "/home/wendy/study/myDataset/image1";
    std::vector<cv::String> image_files;
    // TODO  Boost：:filesystem ,  boost：:algorithm：:ends_with（“jpg”);

    cv::glob(pattern_jpg, image_files); //dir.GetListFiles
    vector<Mat> readimage;
    vector<Mat> fiximage;

    string IntrinsicsPath = "/home/wendy/study/learnSLAM/test/camera_intrinsic_1.yml";

    if (image_files.size() == 0) {
        std::cout << "No image files[jpg]" << std::endl;
        exit(-1);

    }
    cout << image_files.size() << endl;



    for (int frame = 0; frame < image_files.size(); ++frame) {

        vector<string> v;
        string s = image_files[frame];
//        split(v, s, boost::is_any_of("."));
//        if(v.back() != "bmp")
//            continue;

        bool flag = false;
        flag = boost::algorithm::ends_with(s, "bmp");
        if(!flag)
            continue;
        Mat image = cv::imread(image_files[frame]);
        readimage.push_back(image);
        imshow("1", image);
        waitKey(1);

    }
    cout << readimage.size() << endl;

    Size imageSize;
    Mat	cameraMatrix, distCoeffs;
    ReadIntrinsics(cameraMatrix, distCoeffs, imageSize, IntrinsicsPath.c_str());
    for (int i = 0; i <readimage.size() ; ++i) {
        Mat	map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(),
                                imageSize, CV_16SC2, map1, map2);
        Undistort_img(map1, map2, readimage[i]);
        // undistort（img , K, D, ...)
    }

    return 0;
}




void ReadIntrinsics(Mat &cameraMatrix, Mat &distCoeffs, Size &imageSize, const char *IntrinsicsPath)
{
    cv::FileStorage readfs;

    bool FSflag = readfs.open(IntrinsicsPath, FileStorage::READ);
    if (FSflag == false)
        cout << "Cannot open the file" << endl;
    readfs["camera_matrix"] >> cameraMatrix;
    readfs["distortion_coefficients"] >> distCoeffs;
    readfs["image_width"] >> imageSize.width;
    readfs["image_height"] >> imageSize.height;

    cout << cameraMatrix << endl << distCoeffs << endl << imageSize << endl;

    readfs.release();
}

void Undistort_img(Mat map1, Mat map2, Mat nowimage)
{
    Mat img1=nowimage;
    Mat img2;
    Mat dst;
    if (img1.empty()) cout << "Cannot open the image" << endl;
    remap(img1, img2, map1, map2, INTER_LINEAR);

// 	imwrite("fiximage.jpg", img2);

//    mergeImg(dst, img1, img2);
    // opencv 拼接 cv::hconcat()， cv::vconcat()
    hconcat(img1, img2, dst);
    imshow("result", dst);
    waitKey(30);    // 1000ms = 1s / 30ms = 33.3333333hz
}

void mergeImg(Mat & dst,const Mat &src1, const Mat &src2)
{
    int rows = src1.rows;
    int cols = src1.cols+src2.cols;
    CV_Assert(src1.type () == src2.type ());
    dst.create (rows,cols,src1.type ());
    src1.copyTo (dst(Rect(0,0,src1.cols,src1.rows)));
    src2.copyTo (dst(Rect(src1.cols,0,src2.cols,src2.rows)));
}



