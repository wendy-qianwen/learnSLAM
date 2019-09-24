//
// Created by wendy on 2019/9/24.
//

#include <opencv2/opencv.hpp>
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

string imageFolder = "/home/wendy/study/myDataset/image3";
string outputFile = "result.xml";
Size g_boardSize(11, 8);    // 棋盘格内点数(width/cols, height/rows)
float g_squareSize = 30;    // 棋盘格宽度[mm]


void readImage(const string dataFolder, vector<string>& files){
    bf::path path(dataFolder);
    if (!bf::exists(path)) {
        cerr << "[Main] Data folder doesn't exist!" << endl;
        return;
    }

    bf::directory_iterator end_iter;
    for(bf::directory_iterator iter(path); iter != end_iter; ++iter){
        if (bf::is_directory(iter->status()))
            continue;
        if (bf::is_regular_file(iter->status()))
            files.push_back(iter->path().string());
    }


}

bool runCalibration(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                    vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                    vector<float> &reprojErrs)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);  // eye(m,n)  m*n 的单位矩阵
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f>> objectPoints(1);
    for (int i = 0; i < g_boardSize.height; ++i)
        for (int j = 0; j < g_boardSize.width; ++j)
            objectPoints[0].push_back(Point3f(j * g_squareSize, i * g_squareSize, 0));

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    // Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,  // 返回值为重投影误差
                                 rvecs, tvecs); //calibrateCamera(世界坐标系中的点 对应的图像点 图像的大小 内参数矩阵 畸变矩阵 旋转向量 位移向量)

    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);



    return ok;
}

static void saveCameraParams(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs)
{
    FileStorage fs(outputFile, FileStorage::WRITE);


    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << g_boardSize.width;
    fs << "board_height" << g_boardSize.height;
    fs << "square_size" << g_squareSize;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
}


int main(int argc, char *argv[]) {
    vector<string> fullImages;
    readImage(imageFolder, fullImages);
    if (fullImages.empty()) {
        cerr << "[3] Not image data in the folder!" << endl;
        return -1;
    } else
        cout << "[3] Read " << fullImages.size() << " files in the folder." << endl;

    Mat cameraMatrix, distCoeffs;        // 待求 内参 和 畸变系数
    vector<vector<Point2f>> allCorners;  // 所有棋盘格角点

    //! 1.输入图像并检测棋盘格角点
    Mat imageOut;
    for (int i = 0; i < fullImages.size(); ++i) {

        Mat image = imread(fullImages[i], CV_LOAD_IMAGE_GRAYSCALE);


        cvtColor(image, imageOut, COLOR_GRAY2BGR);

        vector<Point2f> corners;
        bool found = findChessboardCorners(image, g_boardSize, corners, /*CALIB_CB_FILTER_QUADS +*/
                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cornerSubPix(image, corners, Size(5, 5), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.0001));
            drawChessboardCorners(imageOut, g_boardSize, Mat(corners), found);
            allCorners.push_back(corners);
            cout << "Find " << corners.size() << " corners in image " << i << endl;
        }
        resize(imageOut, imageOut, Size(imageOut.cols/2, imageOut.rows/2));
        imshow("Current Image Corners", imageOut);
        waitKey(10);
    }
    cout << "Detected " << allCorners.size() << " images with chessboard corners." << endl;

    //! 2.计算标定结果
    Size imageSize = imageOut.size();
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    bool ok = runCalibration(imageSize, cameraMatrix, distCoeffs, allCorners, rvecs, tvecs,
                             reprojErrs);


    //! 3.根据标定结果校正畸变,并保存标定结果
    if (ok) {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                imageSize, CV_16SC2, map1, map2);

        for (size_t i = 0; i < fullImages.size(); i++) {
            view = imread(fullImages[i], IMREAD_COLOR);
            if (view.empty()){
                cerr << "no photo" << endl;
                continue;
            }


            remap(view, rview, map1, map2, INTER_LINEAR);
//          undistort(temp, image, cameraMatrix, distCoeffs);
            Mat dst;
            resize(rview, dst, Size(rview.cols/2, rview.rows/2));
            imshow("Image Undistortion", dst);
            waitKey(30);

        }

        saveCameraParams(imageSize, cameraMatrix, distCoeffs);
        cout << "Save calibration output to " << outputFile << endl;
    }

    return 0;
}