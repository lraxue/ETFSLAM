//
// Created by feixue on 17-8-27.
//

#ifndef SSLAM_UTILS_H
#define SSLAM_UTILS_H

#include <Converter.h>
#include <vector>
#include <string>
#include <iomanip>
#include <dirent.h>

#include <glog/logging.h>

using namespace std;

inline void LoadImages(const std::string &strImageSequencePath, std::vector<std::string> &vstrImageLeft,
                       std::vector<std::string> &vstrImageRight, std::vector<double> &vTimes)
{
    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir((strImageSequencePath + "/image_0/").c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(-1);
    }


    std::vector<std::string> vImageName;
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)    ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8)
        {
            vImageName.push_back((string) ptr->d_name);
//            LOG(INFO) << ptr->d_name;
        }
    }

    closedir(dir);

    std::sort(vImageName.begin(), vImageName.end());
    vTimes.resize(vImageName.size());
    vstrImageLeft.resize(vImageName.size());
    vstrImageRight.resize(vImageName.size());

    for (int i = 0, iend = vImageName.size(); i < iend; ++i)
    {
        vstrImageLeft[i] = strImageSequencePath + "/image_0/" + vImageName[i];
        vstrImageRight[i] = strImageSequencePath + "/image_1/" + vImageName[i];

        vTimes.push_back(0.01 * i);
    }

    LOG(INFO) << "Load " << vstrImageLeft.size() << " images from " << strImageSequencePath;
}


inline int Rotation2Quaternion(const std::string& sourcefile, const std::string& targetfile)
{
    std::fstream file1(sourcefile.c_str());
    std::fstream file2(targetfile.c_str(), ios::out | ios::app);
    if (!file1.is_open())
    {
        LOG(INFO) << "Open file " << sourcefile << " error.";
        return -1;
    }

    if (!file2.is_open())
    {
        LOG(INFO) << "Open file " << targetfile << " error.";
        return -1;
    }

    std::string line;
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    int n = 0;
    file2 << fixed;
    while(getline(file1, line))
    {
        std::stringstream ss;
        ss << line;

        cout << line << endl;

        ss >> T.at<float>(0, 0) >> T.at<float>(0, 1) >> T.at<float>(0, 2) >> T.at<float>(0, 3)
           >> T.at<float>(1, 0) >> T.at<float>(1, 1) >> T.at<float>(1, 2) >> T.at<float>(1, 3)
           >> T.at<float>(2, 0) >> T.at<float>(2, 1) >> T.at<float>(2, 2) >> T.at<float>(2, 3);

        cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T.rowRange(0, 3).col(3);

        std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

        file2 << setprecision(6) << n*0.1 << " " << setprecision(9)
              << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
              << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        n++;

        line.clear();
        ss.clear();
    }

    file1.close();
    file2.close();

    return n;
}

inline int AddTimeStamps(const std::string& sourcefile, const std::string& targetfile)
{
    std::fstream file1(sourcefile.c_str());
    std::fstream file2(targetfile.c_str(), ios::out | ios::app);
    if (!file1.is_open())
    {
        LOG(INFO) << "Open file " << sourcefile << " error.";
        return -1;
    }

    if (!file2.is_open())
    {
        LOG(INFO) << "Open file " << targetfile << " error.";
        return -1;
    }

    std::string line;
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    int n = 0;
    file2 << fixed;
    while(getline(file1, line))
    {
        cout << line << endl;

        file2 << setprecision(6) << n*0.1 << " " << line << endl;
        n++;

        line.clear();
    }

    file1.close();
    file2.close();

    return n;
}

#endif //SSLAM_UTILS_H
