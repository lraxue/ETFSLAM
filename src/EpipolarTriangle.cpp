//
// Created by feixue on 17-8-27.
//

#include <EpipolarTriangle.h>
#include <include/Config.h>

namespace ORB_SLAM2
{
    unsigned long EpipolarTriangle::mnNext = 0;

    float EpipolarTriangle::mSpatioTheta = 1.f;
    float EpipolarTriangle::mAngleTheta = 1.f;
    float EpipolarTriangle::alphaR = 1.f;
    float EpipolarTriangle::alphaA = 1.f;
    float EpipolarTriangle::alphaS = 1.f;

    float EpipolarTriangle::mImageWidth = 0.f;
    float EpipolarTriangle::mImageHeight = 0.f;

    bool EpipolarTriangle::bInitialization = false;

    EpipolarTriangle::EpipolarTriangle()
    {}

    EpipolarTriangle::EpipolarTriangle(const unsigned long &frameId, const cv::Mat &X, const cv::Mat &Cl,
                                       const cv::Mat &Cr) : mAngle1(0.0), mAngle2(0.0), mAngle3(0.0)
    {
        mnId = mnNext++;

        mnFrameId = frameId;

        // Initialize the three vertexes
        mX = X.clone();
        mCl = Cl.clone();
        mCr = Cr.clone();

        // Compute normal and distance
        // ComputeNormalAndDistance();

        // Compute three angles
        ComputeThreeAngles();

        // Compute intrinsic uncertainty
//        ComputeUncertainty();

    }

    EpipolarTriangle::EpipolarTriangle(const unsigned long &frameId, const cv::Mat &X, const cv::Mat &Cl,
                                       const cv::Mat &Cr, const cv::KeyPoint &keyLeft, const cv::KeyPoint &keyRight,
                                       const float &depth, const float &uRight)
            : mX(X.clone()), mCl(Cl.clone()), mCr(Cr.clone()), mKeyLeft(keyLeft), mKeyRight(keyRight), mDepth(depth),
              muRight(uRight)
    {
        mnId = mnNext++;
        mnFrameId = frameId;

        if (!bInitialization)
        {
//            mSpatioTheta = Config::mThMatch;
//            mAngleTheta = Config::mThAngle;
            mImageWidth = Config::mImageWidth;
            mImageHeight = Config::mImageHeight;
            
            mSpatioTheta = sqrt(mImageWidth * mImageWidth + mImageHeight * mImageHeight);
            mAngleTheta = 20;

            alphaS = 0.3;
            alphaR = 0.5;
            alphaA = 0.2;

            bInitialization = true;
        }

        // Compute normal and distance
        ComputeNormalAndDistance();

        // Compute three angles
        ComputeThreeAngles();

        // Compute uncertainty
        ComputeUncertainty();

    }

    EpipolarTriangle::~EpipolarTriangle()
    {

    }

    float EpipolarTriangle::ComputeDistance(const cv::Mat &x3Dw)
    {
        return abs(x3Dw.dot(mNormal) + mD);
    }

    void EpipolarTriangle::Transform(const cv::Mat &T)
    {
        if (T.empty())
            return;

        // Extract R and t
        const cv::Mat R = T.rowRange(0, 3).colRange(0, 3).clone();
        const cv::Mat t = T.rowRange(0, 3).col(3);

        Transform(R, t);
    }

    void EpipolarTriangle::Transform(const cv::Mat &R, const cv::Mat &t)
    {
        // Update normal vector and distance to the origin
        // n' = R * n
        // d' = -t.t() * n' + d

        if (R.empty() || t.empty())
            return;

        mNormal = R * mRawNormal;
        mD = fabs(-mNormal.dot(t) + mRawD);
    }

    float EpipolarTriangle::ComputeDistanceToVertex(const cv::Mat &x3Dw, const int &idx)
    {
        if (idx == 0)
            return x3Dw.dot(mX);
        else if (idx == 1)
            return x3Dw.dot(mCl);
        else if (idx == 2)
            return x3Dw.dot(mCr);
    }

    void EpipolarTriangle::ComputeNormalAndDistance()
    {
        cv::Mat X12 = mCl - mX;
        cv::Mat X13 = mCl - mCr;

        cv::Mat normal = X12.cross(X13);

        mNormal = normal / cv::norm(normal);
        mD = cv::norm(mX.t() * mNormal);

        mRawNormal = mNormal;
        mRawD = mD;
    }

    void EpipolarTriangle::ComputeUncertainty()
    {
        // Compute feature uncertainty
        const float rl = mKeyLeft.response / 255.f;
        const float rr = mKeyRight.response / 255.f;
        mResponse = (rl + rr) / 2.f;

        // Spatial distribution 
        const float dlx = mKeyLeft.pt.x - mImageWidth/2.0;
        const float dly = mKeyLeft.pt.y - mImageHeight/2.0;
        const float drx = mKeyRight.pt.x - mImageWidth / 2.0;
        const float dry = mKeyRight.pt.y - mImageHeight / 2.0;
        mSpatioRatio = ((sqrt(dlx*dlx + dly*dly) + sqrt(drx*drx + dry*dry)) / 2.0) / mSpatioTheta;

        // Observation uncertainty
        mAngleRatio = mAngle1 / mAngleTheta;
        // Fuse uncertainty
        mFusedUncertainty = std::exp(alphaR * mResponse + alphaA * mAngleRatio - alphaS * mSpatioRatio);
    }

    void EpipolarTriangle::ComputeThreeAngles()
    {
        cv::Mat e12 = mX - mCl;
        cv::Mat e13 = mX - mCr;
        cv::Mat e23 = mCl - mCr;
        double len12 = cv::norm(e12);
        double len13 = cv::norm(e13);
        double len23 = cv::norm(e23);

        double cos2 = (len12 * len12 + len23 * len23 - len13 * len13) / (2 * len12 * len23);
        double cos1 = (len12 * len12 + len13 * len13 - len23 * len23) / (2 * len12 * len13);
        double cos3 = (len13 * len13 + len23 * len23 - len12 * len12) / (2 * len13 * len23);

        mAngle1 = (float) (std::acos(cos1) * 180.f / CV_PI);
        mAngle2 = (float) (std::acos(cos2) * 180.f / CV_PI);
        mAngle3 = (float) (std::acos(cos3) * 180.f / CV_PI);

//        LOG(INFO) << "len12: " << len12 << " ,len13: " << len13 << " ,len23: " << len23;
//        LOG(INFO) << "angle1: " << mAngle1 << " ,angle2: " << mAngle2 << " ,angle3: " << mAngle3  << " ,total: " << mAngle1 + mAngle2 + mAngle3;

    }


    float EpipolarTriangle::Angle1() const
    {
        return mAngle1;
    }

    float EpipolarTriangle::Angle2() const
    {
        return mAngle2;
    }

    float EpipolarTriangle::Angle3() const
    {
        return mAngle3;
    }

    float EpipolarTriangle::Uncertainty() const
    {
        return mFusedUncertainty;
    }

    cv::Point2f EpipolarTriangle::PointLeft() const
    {
        return mKeyLeft.pt;
    }

    cv::Point2f EpipolarTriangle::PointRight() const
    {
        return mKeyRight.pt;
    }

    cv::Mat EpipolarTriangle::DescriptorLeft() const
    {
        return mDescLeft.clone();
    }

    cv::Mat EpipolarTriangle::DescriptorRight() const
    {
        return mDescRight.clone();
    }
}