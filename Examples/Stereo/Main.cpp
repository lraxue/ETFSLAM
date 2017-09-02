/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include <utils.h>
#include<System.h>
#include <include/Values.h>
#include <include/Config.h>

using namespace ORB_SLAM2;
using namespace std;


int main(int argc, char** argv)
{
    if (argc !=2)
    {
        LOG(ERROR) << "./ETFSLAM dataset";
        return -1;
    }

    int settingId[] = {0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 2};
    int loopI = std::stoi(std::string(argv[1]));

    LOG(INFO) << "Process dataset " << loopI;

    int nKittiImageIdx = 10;
    int nKittiSettingIdx = 2;
    const string settingFile = strKittiSettingPaths[settingId[loopI]]; // nKittiSettingIdx
    const string vocFile = strVocFile;
    const string imagePath = strKittiImagePaths[loopI]; // nKittiImageIdx
    const string ETFResultPath = "./etf_results/";
    const string ORBResultPath = "./orb_results/";

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;

    LoadImages(imagePath, vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    Config::LoadParameters(settingFile);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(vocFile, settingFile, System::STEREO, false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << "Process dataset " << loopI << " -------" << endl;
//    cout << "Start processing sequence ..." << endl;
//    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    int nEnd = nImages;
    cv::Mat imLeft, imRight;
    for (int ni = 0; ni < nEnd; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe);

//        cout << endl;

    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
//    SLAM.SaveTrajectoryKITTI("Uncertainty_Kitti_00.txt");
//    SLAM.SaveMap("map_kitti_00.txt");
    SLAM.SaveTrajectoryTUM(ETFResultPath + strETFResults[loopI]); // nKittiImageIdx

    return 0;
}

