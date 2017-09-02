//
// Created by feixue on 17-8-27.
//

#ifndef SSLAM_VALUES_H
#define SSLAM_VALUES_H

#include <string>

using namespace std;

namespace ORB_SLAM2
{
    const string strVocFile = "../Vocabulary/ORBvoc.bin";
    const string ProjectPath = "/home/feixue/Research/Code/SLAM/SSLAM/";
    const string strSettingFilePKUDesk = "Stereo/pku-desk.yaml";
    const string strImagePathPKUDesk = "/home/feixue/Research/Dataset/data/Desk/images/02/";

    // Kitti dataset
    const string strKittiPoseGT[11] = {"/home/feixue/Research/Dataset/Kitti/dataset/poses/00.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/01.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/02.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/03.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/04.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/05.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/06.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/07.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/08.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/09.txt",
                                       "/home/feixue/Research/Dataset/Kitti/dataset/poses/10.txt"
    };

    const string strKittiPoseQuaternion[11] = {"/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_00.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_01.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_02.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_03.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_04.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_05.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_06.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_07.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_08.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_09.txt",
                                               "/home/feixue/Research/Dataset/Kitti/dataset/poses/qua_10.txt"
    };

    std::string strETFResults[11] = {"etf_kitti_quaxzy_00.txt", "etf_kitti_quaxzy_01.txt",
                                 "etf_kitti_quaxzy_02.txt", "etf_kitti_quaxzy_03.txt",
                                 "etf_kitti_quaxzy_04.txt", "etf_kitti_quaxzy_05.txt",
                                 "etf_kitti_quaxzy_06.txt", "etf_kitti_quaxzy_07.txt",
                                 "etf_kitti_quaxzy_08.txt", "etf_kitti_quaxzy_09.txt",
                                 "etf_kitti_quaxzy_10.txt"};

    std::string strORBResults[11] = {"orb_kitti_quaxzy_00.txt", "orb_kitti_quaxzy_01.txt",
                                     "orb_kitti_quaxzy_02.txt", "orb_kitti_quaxzy_03.txt",
                                     "orb_kitti_quaxzy_04.txt", "orb_kitti_quaxzy_05.txt",
                                     "orb_kitti_quaxzy_06.txt", "orb_kitti_quaxzy_07.txt",
                                     "orb_kitti_quaxzy_08.txt", "orb_kitti_quaxzy_09.txt",
                                     "orb_kitti_quaxzy_10.txt"};

    std::string strKittiImagePaths[11] = {"/home/feixue/Research/Dataset/Kitti/dataset/sequences/00",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/01",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/02",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/03",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/04",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/05",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/06",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/07",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/08",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/09",
                                     "/home/feixue/Research/Dataset/Kitti/dataset/sequences/10"};

    std::string strKittiSettingPaths[3] = {"../Examples/Stereo/KITTI00-02.yaml",
                                   "../Examples/Stereo/KITTI03.yaml",
                                   "../Examples/Stereo/KITTI04-12.yaml"};
}
#endif //SSLAM_VALUES_H
