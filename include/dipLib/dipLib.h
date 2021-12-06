//
// Created by stdcat on 12/2/21.
//

#ifndef DIP_WS_DIPLIB_H
#define DIP_WS_DIPLIB_H

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#define _SIGN(x) (x)<0? -1 : ( (x)>0 ? 1 : 0)

namespace dipLib{
    using namespace Eigen;
    using namespace cv;
    using namespace std;

    /* typedef enum*/
    typedef enum {

        HSeg_NONE = 0,
        HSeg_OK

    }HSeg_e;

    typedef enum {

        Ctrl_PLAN = 0,
        Ctrl_NAV,
        Ctrl_STOPPED,
        Ctrl_NONE

    }Ctrl_e;

    /* typedef struct */
    typedef struct {

        Vector2d origin;
        double coef;
        double consAngle;

    }OriginParam_t;

    typedef struct {

        HSeg_e state;
        Vector2d targetPoint;
        double targetAngle;

    }HSeg_RAW;

    typedef struct {

        Ctrl_e state;

        double curAngle;
        Vector2d position;

        double endAngle;
        Vector2d endPosition;

    }Ctrl_RAW;


    /* const params */
    const double pi = 3.1415926;

    /* functions */
    double angleMainValue( double rad ){
        while (rad < -pi) rad += 2*pi;
        while (rad > pi) rad -= 2*pi;
        return rad;
    }

    /* class */
    class dipColorSegment{
    protected:
        int hMin;
        int hMax;
        int sMin;
        int sMax;
        int vMin;
        int vMax;
    public:

        dipColorSegment(
                int _hMin,
                int _hMax,
                int _sMin,
                int _sMax,
                int _vMin,
                int _vMax
                );
        Mat segment(Mat bgr);
        void setSegParam(
                int _hMin,
                int _hMax,
                int _sMin,
                int _sMax,
                int _vMin,
                int _vMax
                );
        void setSegParam( VectorXi param );
    };

    class HCharSeg:
        public dipColorSegment
{
private:
    inline static bool rectCMP(pair<Rect, int > &a, pair<Rect, int > &b){
        return  a.first.width*a.first.height > b.first.width*b.first.height;
    }

    inline static bool areaCMP(pair<int, int > &a, pair<int, int > &b){
        return  a.first > b.first;
    }

    /* Gamma Correction*/
    void GammaCorrection(Mat src, Mat &dst, float fGamma);

    /* get the center of each contour */
    Vector2d getCenter(vector<Point> li);

    int gammaPercent;

    HSeg_RAW raw;

public:

    HCharSeg();

    void load(Mat src);

    HSeg_RAW getRaw();

};


}

#endif //DIP_WS_DIPLIB_H
