/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);


int main(int argc, char **argv)
{
    //输入文件合规校验
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }
    const int num_seq = (argc-3)/2;  //输入序列的个数，有3个是肯定有的（可执行文件/字典/配置），剩下的成对出现（图像（包括IMU）路径/时间戳）
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);  //判断 图像路径与时间戳是否成对出现，成对出现=0,否则=1
    string file_name;
    if (bFileName) //打印不成对的文件名
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;                                    //输入序列num
    vector< vector<string> > vstrImageLeft;     //存放左图像每一张的路径
    vector< vector<string> > vstrImageRight;    //存放右图像每一张的路径
    vector< vector<double> > vTimestampsCam;    //摄像头时间戳
    vector< vector<cv::Point3f> > vAcc, vGyro;  //加速度计和陀螺仪数据
    vector< vector<double> > vTimestampsImu;    //IMU时间戳
    vector<int> nImages;                        //单个序列中图像数量
    vector<int> nImu;                           //单个序列中IMU数据数量
    vector<int> first_imu(num_seq,0);  //初始化为num_seq个为0的向量
    //调整上述向量空间大小
    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0; //统计每个seq一起总的加载图像数量

    //开始按seq加载图像和IMU数据
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);              //序列路径
        string pathTimeStamps(argv[(2*seq) + 4]);       //时间戳路径
        string pathCam0 = pathSeq + "/mav0/cam0/data";      //左图像路径
        string pathCam1 = pathSeq + "/mav0/cam1/data";      //右图像路径
        string pathImu = pathSeq + "/mav0/imu0/data.csv";   //IMU数据路径

        //加载图像文件路径（而非数据）
        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        //加载IMU数据
        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();   //此序列中图像数量
        tot_images += nImages[seq];             //seq之间的求和统计
        nImu[seq] = vTimestampsImu[seq].size(); //此序列中IMU数据量

        //错误提示
        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        //相机与IMU数据第一个头部对齐
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

    // Read rectification parameters
    //尝试加载镜头参数，提供错误提示
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    //用于统计追踪的运行时间
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17); //设置输出浮点精度

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //初始化class
    ORB_SLAM3::System SLAM(argv[1],     //字典
                           argv[2],  //参数
                           ORB_SLAM3::System::IMU_STEREO,   //传感器类型
                           true);   //可视化viewer

    //基于OpenCV加载图像、IMU数据加载
    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas; //装载IMU测量值
        //这几个参数都没啥用
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;

        //images loop
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read left and right images from file
            //imread加载左右图像
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }
            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];    //当前帧图像的时间戳

            // Load imu measurements from previous frame
            //加载IMU数据
            vImuMeas.clear();   //元素清零

            if(ni>0)
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    //在图像时间戳之前产生的IMU数据全部加载
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,   //三轴加速度计数据
                                                             vAcc[seq][first_imu[seq]].y,
                                                             vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,  //三轴陀螺仪数据
                                                             vGyro[seq][first_imu[seq]].y,
                                                             vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]])); //IMU数据时间戳
                    first_imu[seq]++;
                }


            #ifdef COMPILEDWITHC11  //计时开始
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
            #endif

            // Pass the images to the SLAM system
            //开始跟踪
            SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

            #ifdef COMPILEDWITHC11  //计时停止
                    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
            #endif

            #ifdef REGISTER_TIMES
                    t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
                    SLAM.InsertTrackTime(t_track);
            #endif
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count(); //计算时间差
            vTimesTrack[ni]=ttrack; //每一对图像的时间汇总

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)   //图像还没到本序列的最后一张
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)   //图像是最后一张
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)    //作用是？
                usleep((T-ttrack)*1e6); // 1e6
        }

        if(seq < num_seq - 1)   //如果还有序列继续加载运行
        {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }

    }
    // Stop all threads 停止所有线程
    SLAM.Shutdown();

    // Save camera trajectory 储存结果
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

//加载图像文件路径xxxx.png（而非图像数据）
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);  //至少可以容纳5000个元素
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

//加载IMU数据
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            //由此可知IMU数据文件的结构
            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
