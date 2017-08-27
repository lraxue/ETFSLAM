//
// Created by feixue on 17-8-27.
//

#include <Config.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{
    // static parameters
    float Config::fx = 0.0f;;
    float Config::fy = 0.0f;
    float Config::cx = 0.0f;
    float Config::cy = 0.0f;
    float Config::mb = 0.0f;
    float Config::mbf = 0.0f;
    float Config::fps = 0.0f;
    float Config::iniThFAST = 0.0f;
    float Config::minThFAST = 0.0f;
    float Config::mfScaleFactor = 0.0f;
    int Config::mnLevels = 0;
    int Config::mnFeatures = 0;
    float Config::mThDepth = 0.0f;
    float Config::mThAngle = 0.0f;
    float Config::mThMatch = 0.0f;

    // For Viewer
    float Config::mKeyFrameSize = 0.0f;
    float Config::mKeyFrameLineWidth = 0.0f;
    float Config::mGraphLineWidth = 0.0f;
    float Config::mPointSize = 0.0f;
    float Config::mCameraSize = 0.0f;
    float Config::mCameraLineWidth = 0.0f;

    float Config::mImageWidth = 1226.0f;
    float Config::mImageHeight = 370.0f;

    float Config::mViewpointX = 0.0f;
    float Config::mViewpointY = 0.0f;
    float Config::mViewpointZ = 0.0f;
    float Config::mViewpointF = 0.0f;

    // Self-defined parameters
    int Config::mnGridCols = 64;
    int Config::mnGridRows = 36;

    // Load parameters from file
    void Config::LoadParameters(const std::string& strSettingFile)
    {
        cv::FileStorage fs = cv::FileStorage(strSettingFile.c_str(), cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            LOG(ERROR) << "Open setting file " << strSettingFile << " error.";
            exit(-1);
        }

        // Calibration parameters
        fx = fs["Camera.fx"];
        fy = fs["Camera.fy"];
        cx = fs["Camera.cx"];
        cy = fs["Camera.cy"];
        mbf = fs["Camera.bf"];
        mb = mbf / fx;

        fps = fs["Camera.fps"];

        mThDepth = fs["ThDepth"];

        mImageHeight = fs["Camera.height"];
        mImageWidth = fs["Camera.width"];

        // Feature parameters
        mnFeatures = fs["ORBextractor.nFeatures"];
        mnLevels = fs["ORBextractor.nLevels"];
        mfScaleFactor = fs["ORBextractor.scaleFactor"];
        iniThFAST = fs["ORBextractor.iniThFAST"];
        minThFAST = fs["ORBextractor.minThFAST"];

        mKeyFrameSize = fs["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fs["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fs["Viewer.GraphLineWidth"];
        mPointSize = fs["Viewer.PointSize"];
        mCameraSize = fs["Viewer.CameraSize"];
        mCameraLineWidth = fs["Viewer.CameraLineWidth"];

        mViewpointX = fs["Viewer.ViewpointX"];
        mViewpointY = fs["Viewer.ViewpointY"];
        mViewpointZ = fs["Viewer.ViewpointZ"];
        mViewpointF = fs["Viewer.ViewpointF"];

        // Angle for observation uncertainty
        mThAngle = fs["ThAngle"];

        // Match threshold for matching uncertainty
        mThMatch = std::pow(mfScaleFactor, mnLevels);  // Search range

        LOG(INFO) << "fx: " << fx << " fy: " << fy << " cx: " << cx << " cy: " << cy << " mbf: " << mbf;
        LOG(INFO) << "fps: " << fps << " ThDepth: " << mThDepth << " " << " scale factor: " << mfScaleFactor << " levels: " << mnLevels << " features: " << mnFeatures;
        LOG(INFO) << "ThAngle: " << mThAngle << " ThMatch: " << mThMatch;

    }
}