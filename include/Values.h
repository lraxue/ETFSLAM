//
// Created by feixue on 17-8-27.
//

#ifndef SSLAM_VALUES_H
#define SSLAM_VALUES_H

#include <string>

using namespace std;

namespace ORB_SLAM2
{
    const string strVocFile = "../Vocabulary/ORBvoc.txt";
    const string ProjectPath = "/home/feixue/Research/Code/SLAM/SSLAM/";
    const string strSettingFilePKUDesk = "Stereo/pku-desk.yaml";
    const string strImagePathPKUDesk = "/home/feixue/Research/Dataset/data/Desk/images/02/";

    // KITTI dataset
    const string strSettingFileKitti00_02 = "../Examples/Stereo/KITTI00-02.yaml";
    const string strImagePathKITTI00 = "/home/feixue/Research/Dataset/Kitti/dataset/sequences/00/";
    const string strImagePathKITTI01 = "/home/feixue/Research/Dataset/Kitti/dataset/sequences/01/";
    const string strImagePathKITTI02 = "/home/feixue/Research/Dataset/Kitti/dataset/sequences/02/";


    const string strSettingFileKitti03 = "Stereo/KITTI03.yaml";
    const string strImagePathKITTI03 = "/home/feixue/Research/Dataset/Kitti/dataset/sequences/03/";

    const string strSettingFileKitti04 = "Stereo/KITTI04-12.yaml";
    const string strImagePathKITTI04 = "/home/feixue/Research/Dataset/Kitti/dataset/sequences/04/";

    const string strGTFileKitti00 = "/home/feixue/Research/Dataset/Kitti/dataset/poses/00.txt";
    const string strGTFileKitti01 = "/home/feixue/Research/Dataset/Kitti/dataset/poses/01.txt";
    const string strGTFileKitti02 = "/home/feixue/Research/Dataset/Kitti/dataset/poses/02.txt";
    const string strGTFileKitti03 = "/home/feixue/Research/Dataset/Kitti/dataset/poses/03.txt";
    const string strGTFileKitti04 = "/home/feixue/Research/Dataset/Kitti/dataset/poses/04.txt";
    const string strGTFileKitti05 = "/home/feixue/Research/Dataset/Kitti/dataset/poses/05.txt";
}
#endif //SSLAM_VALUES_H
