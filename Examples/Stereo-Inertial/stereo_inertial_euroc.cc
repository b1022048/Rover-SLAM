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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>// 用于时间统计
#include <ctime>//時間相關函數
#include <sstream>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);//strPathRight : 一個字串，存右相機的資料夾路徑 ， vstrImageRight：一個 vector，存右相機每張圖片的完整路徑

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);


int main(int argc, char **argv)//用 argc 知道邊界，用 argv 讀內容。1.	執行程式，輸入命令引數。argc是argv的長度，也就是命令引數的數量，argv是個字串陣列，存著每個命令引數的內容。
{
    if(argc < 5)//2.	檢查命令引數是否有錯
    {
        cerr << endl << "Usage: ./stereo_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;//幾組資料，扣掉前面的程式名稱、詞貸、設定檔路徑
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)//3.	如果命令引數的最後面還有輸入字串，該輸入字串即為檔名名稱。
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq; //4.	宣告seq：序列迴圈的計數器。
    vector< vector<string> > vstrImageLeft;//vector<type> vectorName 5.	宣告並規劃等等要存放資料的容器大小：
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);
    //6.	對變數進行resize，根據num_seq
    vstrImageLeft.resize(num_seq);//剛宣告時是空的：vstrImageLeft = []，resize後變成vstrImageLeft = [ [], [] ]if num_seq = 2
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";
        //7.開啟時間戳記檔，逐行讀取，每一行就是一張圖片的時間戳記，用它拼出左右相機的圖片完整路徑，同時把時間戳記從奈秒轉成秒，全部存進對應的 vector。
        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);

        //void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        //8.開啟 IMU 的 CSV 檔，逐行讀取，跳過 # 開頭的標頭行，每一行用逗號切割成 7 個數值，分別存入時間戳記、加速度、角速度三個 vector。
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);

        //void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();//紀錄內層大小，也就是每組資料的圖片數量
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();
        //9.	防呆檢查，確認資料都有成功Load。
        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])//10. 找「第一幀影像之前最近的那筆 IMU 資料」。也就是找到第一個時間點超過 cam0 的 IMU，
            first_imu[seq]++;//while會停留在這裡
        first_imu[seq]--; // first imu measurement to be considered
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);//11.	讀取yaml設定檔
    if(!fsSettings.isOpened())//12.	防呆檢查，確認設定檔的路徑有沒有錯。
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;//13.	宣告一個空的vector 設定其大小為所有序列的總幀數(tot_images)
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);//輸出的浮點數都用 17 位有效數字。

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true);//14. 對應到System.h的106行 System.cc 50行
    //System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,const bool bUseViewer, const int initFr, const string &strSequence):
    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;//15.	宣告一個空的 vector vImuMeas，裡面存放 IMU 的資料結構，這個結構在 ImuTypes.h 定義，裡面有加速度、角速度和時間戳記三個成員。
        double t_rect = 0.f;//16. 初始化每個序列會用到的變數並放在for迴圈裡
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)//17. for迴圈 ni：目前在第幾幀（從 0 開始，每個序列都重置）proccIm：總共處理了幾幀（跨序列累加）
        {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())//17.1 防呆檢查，確認圖片路徑有沒有錯。
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())//17.2 防呆檢查，確認圖片路徑有沒有錯。
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];//19. 讀取圖片的時間戳記 把當前幀的時間戳記存到一個簡短的變數名稱裡，方便後面使用。

            // Load imu measurements from previous frame
            vImuMeas.clear();//20	清空 vImuMeas，準備存放這一幀的 IMU 資料。

            if(ni>0)//21.	如果不是第一幀，就把「第一幀影像之前最近的那筆 IMU 資料」到「當前幀的時間戳記」之間的 IMU 資料都存到 vImuMeas 裡。
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));//在ImuTypes.h中
                    first_imu[seq]++;
                }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #endif

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;//23.	紀錄SLAM.TrackStereo()所消耗的時間

            // Wait to load the next frame控制撥放速度
            double T=0;
            if(ni<nImages[seq]-1)//一般幀（不是最後一幀）
                T = vTimestampsCam[seq][ni+1]-tframe;//下一幀的時間戳記 - 當前幀的時間戳記 = 兩幀之間的時間差
            else if(ni>0)// 最後一幀（且不是第一幀）    
                T = tframe-vTimestampsCam[seq][ni-1];//當前幀的時間戳記 - 前一幀的時間戳記 = 兩幀之間的時間差

            if(ttrack<T)//控制播放速度
                usleep((T-ttrack)*1e6); // 1e6
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();//切換到下一個資料集序列。
        }


    }
    // Stop all threads 26.	處理完所有序列的所有幀之後，呼叫 SLAM.Shutdown()，這個函式會請所有的執行緒停止，並等到它們真正停止了才繼續往下走。
    SLAM.Shutdown();


    // Save camera trajectory
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

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;//讀取檔案
    fTimes.open(strPathTimes.c_str());//讀取檔案路徑，c_str()將string轉換為char*，屬於C的寫法
    vTimeStamps.reserve(5000);//預留空間，避免vector在添加元素時頻繁重新分配內存
    vstrImageLeft.reserve(5000);//預留空間，避免vector在添加元素時頻繁重新分配內存
    vstrImageRight.reserve(5000);//預留空間，避免vector在添加元素時頻繁重新分配內存
    while(!fTimes.eof())// eof= end of file。當讀取到文件末尾時，eof()返回true，表示已經沒有更多的數據可供讀取了。
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;//把字串當成一個可以讀寫的資料流
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");//.str()：把緩衝區內容取出為 string 
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

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
        if (s[0] == '#')//uRoC 的 CSV 檔開頭有標頭說明行，以 # 開頭，這裡跳過這些行
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);// 取出逗號前的子字串
                data[count++] = stod(item);// 轉成 double 存入 data[]
                s.erase(0, pos + 1);// 把已處理的部分刪掉
            }
            item = s.substr(0, pos);// 取出最後一個逗號後的子字串
            data[6] = stod(item);// 轉成 double 存入 data[]

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));// 加速度
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));// 角速度
        }
    }
}
