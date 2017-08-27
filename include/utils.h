//
// Created by feixue on 17-8-27.
//

#ifndef SSLAM_UTILS_H
#define SSLAM_UTILS_H

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
    if ((dir = opendir((strImageSequencePath + "image_0/").c_str())) == NULL)
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
        vstrImageLeft[i] = strImageSequencePath + "image_0/" + vImageName[i];
        vstrImageRight[i] = strImageSequencePath + "image_1/" + vImageName[i];

        vTimes.push_back(0.01 * i);
    }

    LOG(INFO) << "Load " << vstrImageLeft.size() << " images from " << strImageSequencePath;
}

#endif //SSLAM_UTILS_H
