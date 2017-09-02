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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include <glog/logging.h>
#include <monitor/Monitor.h>

namespace ORB_SLAM2
{

    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    float Frame::mDepthTheta, Frame::mSpatioTheta;
    float Frame::alphaR, Frame::alphaD, Frame::alphaS;

    Frame::Frame()
    {}

//Copy Constructor
    Frame::Frame(const Frame &frame)
            : mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft),
              mpORBextractorRight(frame.mpORBextractorRight),
              mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
              mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeysLeft(frame.mvKeysLeft),
              mvKeysRight(frame.mvKeysRight), mvuRight(frame.mvuRight), mvMatches(frame.mvMatches), mvpTriangles(frame.mvpTriangles),
              mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
              mDescriptorsLeft(frame.mDescriptorsLeft.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
              mvDepthRatio(frame.mvDepthRatio), mvSpatioRatio(frame.mvSpatioRatio), mvResponseRatio(frame.mvResponseRatio), mvFusedUncertainty(frame.mvFusedUncertainty),
              mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
              mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
              mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
              mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
              mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
              mRGBLeft(frame.mRGBLeft.clone()), mRGBRight(frame.mRGBRight.clone())
    {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j] = frame.mGrid[i][j];

        if (!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }


    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft,
                 ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
                 const float &thDepth)
            : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight),
              mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
              mpReferenceKF(static_cast<KeyFrame *>(NULL))
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        if (imLeft.channels() == 1)
        {
            cv::cvtColor(imLeft, mRGBLeft, CV_GRAY2RGB);
            cv::cvtColor(imRight, mRGBRight, CV_GRAY2RGB);
        }
        else
        {
            mRGBLeft = imLeft.clone();
            mRGBRight = imRight.clone();
        }

        // ORB extraction
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();

        N = mvKeysLeft.size();

        if (mvKeysLeft.empty())
            return;

//        UndistortKeyPoints();

        ComputeStereoMatches();

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvpTriangles.resize(N, static_cast<EpipolarTriangle*>(NULL));
        mvbOutlier = vector<bool>(N, false);


        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
//            ComputeImageBounds(imLeft);
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;


            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mDepthTheta = mThDepth;
            mSpatioTheta = sqrt(mnMaxX*mnMaxX + mnMaxY*mnMaxY) / 2.f;
            alphaR = 1.0;
            alphaD = 0;
            alphaS = 0;

            mbInitialComputations = false;

        }

        mb = mbf / fx;

        AssignFeaturesToGrid();

        ComputeUncertainty();
        // GenerateAllEpipolarTriangles();
    }


    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc,
                 cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            : mpORBvocabulary(voc), mpORBextractorLeft(extractor),
              mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
              mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray);

        N = mvKeysLeft.size();

        if (mvKeysLeft.empty())
            return;

//        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            mnMinX = 0.0f;
            mnMaxX = imGray.cols;
            mnMinY = 0.0f;
            mnMaxY = imGray.rows;

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeysLeft[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if (flag == 0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeysLeft, mDescriptorsLeft);
        else
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
    }

    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0, 3).col(3);
        mOw = -mRcw.t() * mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        const float u = fx * PcX * invz + cx;
        const float v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P - mOw;
        const float dist = cv::norm(PO);

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf * invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    vector<size_t > Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel,
                                            const int maxLevel) const
    {
        vector<size_t > vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int) FRAME_GRID_COLS - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int) FRAME_GRID_ROWS - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysLeft[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }


    void Frame::ComputeBoW()
    {
        if (mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptorsLeft);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }


    void Frame::ComputeStereoMatches()
    {
        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);
        mvMatches.resize(N, -1);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        vector<vector<size_t> > vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for (int iR = 0; iR < Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeysLeft[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptorsLeft.row(iL);

            // Compare descriptor to right keypoints
            for (size_t iC = 0; iC < vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                    continue;

                const float &uR = kpR.pt.x;

                if (uR >= minU && uR <= maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * scaleFactor);
                const float scaledvL = round(kpL.pt.y * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w,
                                                                                     scaledvL + w + 1).colRange(
                        scaleduL - w, scaleduL + w + 1);
                IL.convertTo(IL, CV_32F);
                IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w,
                                                                                          scaledvL + w + 1).colRange(
                            scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                    IR.convertTo(IR, CV_32F);
                    IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave] * ((float) scaleduR0 + (float) bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD)
                {
                    if (disparity <= 0)
                    {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }
                    mvDepth[iL] = mbf / disparity;
                    mvuRight[iL] = bestuR;
                    mvMatches[iL] = (int)bestIdxR;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
                mvMatches[vDistIdx[i].second] = -1;
            }
        }
    }

    cv::Mat Frame::UnprojectStereo(const int &i)
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            const float u = mvKeysLeft[i].pt.x;
            const float v = mvKeysLeft[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
            return mRwc * x3Dc + mOw;
        }
        else
            return cv::Mat();
    }

    EpipolarTriangle* Frame::GetEpipolarTriangle(const int &idx)
    {
        if (idx < 0 || idx >= N)   // out of range
            return static_cast<EpipolarTriangle*>(NULL);

        // Uncertainty
        const float& ResponseLeft = mvKeysLeft[idx].response / 255.f;
        const float& ResponseRight = mvKeysRight[idx].response / 255.f;

        // Vertexes
        cv::Mat Cl = (cv::Mat_<float>(3, 1) << 0.f, 0.f, 0.f);
        cv::Mat Cr = (cv::Mat_<float>(3, 1) << mb, 0.f, 0.f);

        const float z = mvDepth[idx];

        if (z < 0)
            return static_cast<EpipolarTriangle*>(NULL);    // valid depth

        const float u = mvKeysLeft[idx].pt.x;
        const float v = mvKeysLeft[idx].pt.y;

        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;

        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

//        EpipolarTriangle* pNewET =
//                new EpipolarTriangle(mnId, x3Dc, Cl, Cr, sUncertainty);   // EpipolarTriangle in current camera coordinate

        EpipolarTriangle* pNewET =
                new EpipolarTriangle(mnId, x3Dc, Cl, Cr,
                                     mvKeysLeft[idx], mvKeysRight[mvMatches[idx]],
                                     mvDepth[idx], mvuRight[idx]);
        return pNewET;
    }

    int Frame::GenerateAllEpipolarTriangles()
    {
        int nValidET = 0;

        for (int i = 0; i < N; ++i)
        {
            EpipolarTriangle* pET = GetEpipolarTriangle(i);

            if (pET)
            {
                nValidET++;

                mvpTriangles[i] = pET;

//                LOG(INFO) << i << " " << pET->mDepth << " " << pET->mFusedUncertainty;
            }
        }
        return nValidET;
    }

    float Frame::ComputeReprojectionError(const int &idx, bool bStereo)
    {
        if (idx < 0 || idx >= N)
            return -1.f;

        MapPoint* pMP = mvpMapPoints[idx];
        if (!pMP) return -1.f;

        const cv::Mat X3Dw = pMP->GetWorldPos();           // Global pose. Embarrassed!
        const cv::Mat X3Dc = mRcw * X3Dw + mtcw;           // Camera coordinate. Excited!
        const float invz = 1.0 / X3Dc.at<float>(2);
        const float x = X3Dc.at<float>(0);
        const float y = X3Dc.at<float>(1);

        const float u = x * fx * invz + cx;
        const float v = y * fy  * invz + cy;
        const float ur = u - mbf / mvDepth[idx];


        const cv::Point2f pl = mvKeysLeft[idx].pt;

        const float dx = u - pl.x;
        const float dy = v - pl.y;
        const float dr = ur - mvuRight[idx];

        return sqrt(dx*dx + dy*dy + dr*dr);


    }

    float Frame::ComputeReprojectionError(bool bStereo)
    {
        int total_error = 0.0f;
        int cnt = 0;
        for (int i = 0; i < N; ++i)
        {
            float error = ComputeReprojectionError(i, bStereo);
            if (error > 0)
            {
                total_error += error;
                cnt += 1;
            }
        }

        return total_error / cnt;
    }

    void Frame::ComputeUncertainty()
    {
        mvResponseRatio.resize(N, -1.0f);
        mvSpatioRatio.resize(N, -1.0f);
        mvDepthRatio.resize(N, -1.0f);
        mvFusedUncertainty.resize(N, -1.0f);

//        fstream file("uncertainty.txt", ios::in | ios::app);
//        if (!file.is_open())
//        {
//            LOG(ERROR) << "Open uncertainty file error.";
//            exit(-1);
//        }

        for (int i = 0; i < N; ++i)
        {
            if (mvDepth[i] < 0)   // invalid point
                continue;
            if (mvDepth[i] > 1.5*mThDepth)
                continue;

            const int idxR = mvMatches[i];
            // Depth uncertainty
            mvDepthRatio[i] = mvDepth[i] / mDepthTheta;

            // Response uncertainty
            const float rl = mvKeysLeft[i].response / 255.f;
            const float rr = mvKeysRight[idxR].response / 255.f;
            mvResponseRatio[i] = (rl + rr) / 2.f;

            // Spatial distribution
            const float dlx = mvKeysLeft[i].pt.x - mnMaxX / 2.0f;
            const float dly = mvKeysLeft[i].pt.y - mnMaxY / 2.0f;
            const float drx = mvKeysRight[idxR].pt.x - mnMaxX / 2.f;
            const float dry = mvKeysRight[idxR].pt.y - mnMaxY / 2.f;

            const float dmean = (sqrt(dlx*dlx + dly*dly) + sqrt(drx*drx + dry*dry)) / 2.f;
            mvSpatioRatio[i] = dmean / mSpatioTheta;

            mvFusedUncertainty[i] = std::exp(alphaR*mvResponseRatio[i] - alphaD*mvDepthRatio[i] - alphaS*mvSpatioRatio[i]);

//            LOG(INFO) << "Frame: " << mnId << " Key: " << i << " R: " << mvResponseRatio[i] << " S: " << mvSpatioRatio[i] << " D: " << mvDepthRatio[i] << " F: " << mvFusedUncertainty[i];

//            file << mnId << " " << mvResponseRatio[i] << " " << mvSpatioRatio[i] << " " << mvDepthRatio[i] << " " << mvFusedUncertainty[i] << endl;
        }

//        file.close();
    }


    void Frame::Record(const bool bShowKeys, const bool bShowUncertainty, const bool bShowMatch)
    {
        cv::Mat mMatchImg;
        cv::Mat mKeysImg;

        cv::Mat mUFeatureImg;
        cv::Mat mUSpatioImg;
        cv::Mat mUAngleImg;
        cv::Mat mUFuseImg;


        std::vector<cv::KeyPoint> vMatchedKeysLeft;
        std::vector<cv::KeyPoint> vMatchedKeysRight;
        std::vector<cv::KeyPoint> vMatchedKeysRightModifiied;

        std::vector<float> vUncertaintyResponse;
        std::vector<float> vUncertaintySpatio;
        std::vector<float> vUncertaintyAngle;
        std::vector<float> vFuseUncertainty;

        std::vector<std::pair<float, int> > vRawUR;
        std::vector<std::pair<float, int> > vRawUS;
        std::vector<std::pair<float, int> > vRawUA;
        std::vector<std::pair<float, int> > vUFuse;

        int nMatchedPoints = 0;
        for (int i = 0; i < N; ++i)
        {
            if (mvMatches[i] < 0)
                continue;

            EpipolarTriangle* pTr = mvpTriangles[i];

            vMatchedKeysLeft.push_back(mvKeysLeft[i]);
            vMatchedKeysRight.push_back(mvKeysRight[mvMatches[i]]);

            const float& uR = pTr->mResponse;
            const float& uS = pTr->mSpatioRatio;
            const float& uA = pTr->mAngleRatio;
            const float& uFuse = pTr->mFusedUncertainty;

            vRawUR.push_back(std::make_pair(uR, i));
            vRawUS.push_back(std::make_pair(uS, i));
            vRawUA.push_back(std::make_pair(uA, i));
            vUFuse.push_back(std::make_pair(uFuse, i));

            nMatchedPoints++;
        }

        std::sort(vRawUA.begin(), vRawUA.end());
        std::sort(vRawUR.begin(), vRawUR.end());
        std::sort(vRawUS.begin(), vRawUS.end());
        std::sort(vUFuse.begin(), vUFuse.end());


        std::vector<cv::KeyPoint> vMatchedKeysLeftR;
        std::vector<cv::KeyPoint> vMatchedKeysRightR;
        std::vector<cv::KeyPoint> vMatchedKeysLeftA;
        std::vector<cv::KeyPoint> vMatchedKeysRightA;
        std::vector<cv::KeyPoint> vMatchedKeysLeftS;
        std::vector<cv::KeyPoint> vMatchedKeysRightS;
        std::vector<cv::KeyPoint> vMatchedKeysLeftFuse;
        std::vector<cv::KeyPoint> vMatchedKeysRightFuse;

        int nMatchedKeys = 0;

        for (int i = 0; i < 200; ++i)
        {
            int nReverseI = nMatchedPoints - i - 1;
            vUncertaintyResponse.push_back(vRawUR[nReverseI].first);
            int idxF = vRawUR[nReverseI].second;
            vMatchedKeysLeftR.push_back(mvKeysLeft[idxF]);
            vMatchedKeysRightR.push_back(mvKeysRight[mvMatches[idxF]]);

            vUncertaintyAngle.push_back(vRawUA[nReverseI].first);
            int idxA = vRawUA[nReverseI].second;
            vMatchedKeysLeftA.push_back(mvKeysLeft[idxA]);
            vMatchedKeysRightA.push_back(mvKeysRight[mvMatches[idxA]]);

            vUncertaintySpatio.push_back(vRawUS[i].first);
            int idxS = vRawUS[i].second;
            vMatchedKeysLeftS.push_back(mvKeysLeft[idxS]);
            vMatchedKeysRightS.push_back(mvKeysRight[mvMatches[idxS]]);

            vFuseUncertainty.push_back(vUFuse[nReverseI].first);
            int idxFuse = vRawUA[nReverseI].second;
            vMatchedKeysLeftFuse.push_back(mvKeysLeft[idxFuse]);
            vMatchedKeysRightFuse.push_back(mvKeysRight[mvMatches[idxFuse]]);

            const float response = vUncertaintyResponse[nMatchedKeys];
            const float spatio = vUncertaintySpatio[nMatchedKeys];
            const float angle = vUncertaintyAngle[nMatchedKeys];
            const float fuse = vFuseUncertainty[nMatchedKeys];

            LOG(INFO) << "Frame: " << mnId <<
                      " Response: " << response << " "
                      << "Spatio: " << spatio << " "
                      << "Angle: " << angle << " "
                      << "Fuse: " << fuse;

            nMatchedKeys++;
        }

        if (bShowKeys)
        {
            Monitor::DrawKeyPointsOnStereoFrame(mRGBLeft, mRGBRight, vMatchedKeysLeft, vMatchedKeysRight, mKeysImg);
        }

        if (bShowMatch)
        {
            Monitor::DrawMatchesBetweenTwoImages(mRGBLeft, mRGBRight, vMatchedKeysLeft, vMatchedKeysRight, mMatchImg);
        }

        if (bShowUncertainty)
        {
            Monitor::DrawPointsWithUncertaintyByRadius(mRGBLeft, mRGBRight,
                                                       vMatchedKeysLeftR, vMatchedKeysRightR,
                                                       mUFeatureImg, vUncertaintyResponse, std::vector<float>(), true,
                                                       cv::Scalar(0, 0, 255));

            Monitor::DrawPointsWithUncertaintyByRadius(mRGBLeft, mRGBRight,
                                                       vMatchedKeysLeftS, vMatchedKeysRightS,
                                                       mUSpatioImg, vUncertaintySpatio, std::vector<float>(), true,
                                                       cv::Scalar(255, 0, 0));

            Monitor::DrawPointsWithUncertaintyByRadius(mRGBLeft, mRGBRight,
                                                       vMatchedKeysLeftA, vMatchedKeysRightA,
                                                       mUAngleImg, vUncertaintyAngle, std::vector<float>(), true,
                                                       cv::Scalar(0, 97, 255));

            Monitor::DrawPointsWithUncertaintyByRadius(mRGBLeft, mRGBRight,
                                                       vMatchedKeysLeftFuse, vMatchedKeysRightFuse, mUFuseImg,
                                                       vFuseUncertainty, std::vector<float>(), true,
                                                       cv::Scalar(0, 255, 255));
        }

        string strId = to_string(mnId);

        if (!mKeysImg.empty())
        {
            cv::imshow("keys_" + strId, mKeysImg);
        }

        if (!mMatchImg.empty())
        {
            cv::imshow("match_" + strId, mMatchImg);
        }

        if (!mUFeatureImg.empty())
        {
            cv::imshow("response_" + strId, mUFeatureImg);
        }

        if (!mUAngleImg.empty())
        {
            cv::imshow("angle_" + strId, mUAngleImg);
        }

        if (!mUSpatioImg.empty())
        {
            cv::imshow("spatio_" + strId, mUSpatioImg);
        }

        if (!mUFuseImg.empty())
        {
            cv::imshow("fuse_" + strId, mUFuseImg);
        }

        cv::waitKey(0);

    }

} //namespace ORB_SLAM
