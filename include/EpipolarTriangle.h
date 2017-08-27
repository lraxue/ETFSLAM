//
// Created by feixue on 17-8-27.
//

#ifndef ORB_SLAM2_EPIPOLARTRIANGLE_H
#define ORB_SLAM2_EPIPOLARTRIANGLE_H

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{
    class EpipolarTriangle
    {
    public:
        // Constructor functions
        EpipolarTriangle();

        EpipolarTriangle(const unsigned long &frameId, const cv::Mat &X, const cv::Mat &Cl, const cv::Mat &Cr);

        // Extended constructor functions
        EpipolarTriangle(const unsigned long &frameId, const cv::Mat &X, const cv::Mat &Cl, const cv::Mat &Cr,
                         const cv::KeyPoint &keyLeft, const cv::KeyPoint &keyRight, const float &depth,
                         const float &uRight);

        ~EpipolarTriangle();

    public:

        void Transform(const cv::Mat &R, const cv::Mat &t);

        void Transform(const cv::Mat &T);

        /**
         *
         * @param x3Dw a 3D point in world coordinate
         * @return distance to the plane
         */
        float ComputeDistance(const cv::Mat &x3Dw);

        /**
         *
         * @param idx point id of three points
         * @return distance
         */
        float ComputeDistanceToVertex(const cv::Mat &x3Dw, const int &idx = 0);

        // Get three angles of the triangle
        float Angle1() const;

        float Angle2() const;

        float Angle3() const;


        // Get uncertainty
        float Uncertainty() const;

        // Coordinate in left and right image
        cv::Point2f PointLeft() const;

        cv::Point2f PointRight() const;

        // Descriptor in left and right image
        cv::Mat DescriptorLeft() const;

        cv::Mat DescriptorRight() const;

    protected:
        /**
         * Compute three inner angles, as the structure of the triangle
         */
        void ComputeThreeAngles();

        /**
         * Compute normal and distance
         */

        void ComputeNormalAndDistance();

        void ComputeUncertainty();


    public:
        unsigned long mnId;
        static unsigned long mnNext;

        // Id of the Frame that creating this ETriangle
        unsigned long mnFrameId;

        // Basic information
        cv::KeyPoint mKeyLeft, mKeyRight;
        cv::Mat mDescLeft, mDescRight;
        float mDepth;
        float muRight;

        // Uncertainty
        float mResponse;
        float mSpatioRatio;
        float mAngleRatio;
        float mFusedUncertainty;

        // Attributes
        float mAngle1, mAngle2, mAngle3;  // Three angles: top-left-right

        // Uncertainty defined on observation
        float mObservation;

        // Parameters
        static float mSpatioTheta;
        static float mAngleTheta;
        static float alphaR, alphaA, alphaS;
        static bool bInitialization;
        static float mImageWidth, mImageHeight;


        // Parameters after transformation
        cv::Mat mNormal;
        float mD;

        cv::Mat mRawNormal;
        float mRawD;

        // Three points that define the triangle
        cv::Mat mX;
        cv::Mat mCl;
        cv::Mat mCr;
    };
}

#endif //ORB_SLAM2_EPIPOLARTRIANGLE_H
