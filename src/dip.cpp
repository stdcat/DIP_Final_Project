//
// Created by stdcat on 12/2/21.
//

#include "dipLib/dipLib.h"

#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "algorithm"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "boost/thread.hpp"

using namespace dipLib;
using namespace cv;
using namespace Eigen;
using namespace std;


OriginParam_t oParam;

class bezierController{
private:
    ros::NodeHandle n;

    ros::Timer bezierTimer;

    ros::Publisher velPub;

    ros::Subscriber odomSub, navParamSub, bezierParamSub;

    Ctrl_RAW raw;

    geometry_msgs::Twist _tw;

    struct {
        double kp_v;
        double kp_a;
        double kv_a;
    }nav_param;

    struct {
        double klinear;
        double kangular;
    }bezier_param;

    HCharSeg *hseg;

    bool mapOK;
    Mat mapMat;
    double dist;

    void bezierCtrlThread( const ros::TimerEvent &e ){
        HSeg_RAW hraw = hseg->getRaw();

        if(hraw.state == HSeg_NONE || raw.state == Ctrl_STOPPED ){
            raw.state = Ctrl_STOPPED;
            _tw.linear.x = _tw.angular.z = 0;
            velPub.publish(_tw);
            return;
        }


        Vector2d p0, p1, p2, p3;
        Vector2d endP = hraw.targetPoint - oParam.origin;
        endP /= double (oParam.coef*100.0);
        endP.y() *= -1;

        dist = endP.norm();

        static int generateMap = 0;
        if(!generateMap && endP.norm() < 1.75  ){
            generateMap = 1;
            Vector2d endPoint = endP;

            double yaw = raw.curAngle;
            Matrix2d rotateMat;
            rotateMat << cos(-yaw), -sin(-yaw),
                        sin(-yaw), cos(-yaw);
            endPoint = rotateMat * endPoint;
            endPoint += raw.position;
            double endAngle = hraw.targetAngle;
            endAngle += yaw;

            raw.endAngle = endAngle - pi/2;
            raw.endPosition = endPoint;

            ROS_WARN("endx:%lf endy:%lf dir:%lf", endPoint.x(), endPoint.y(), raw.endAngle*180.0/pi );
        }

        if(endP.norm() < 0.25 || raw.state == Ctrl_NAV)
        {
            raw.state = Ctrl_NAV;

            Vector2d toPoint = raw.endPosition - raw.position ;

            double err_distance, err_angle_point, err_angle_vel;
            err_distance = toPoint.norm();
            err_angle_point = angleMainValue(angleMainValue(atan2(toPoint.y(), toPoint.x())) - pi/2)   ;
            err_angle_vel = raw.endAngle - raw.curAngle;

            _tw.linear.x = nav_param.kp_v * err_distance;
            _tw.angular.z = nav_param.kp_a * err_angle_point + nav_param.kv_a * err_angle_vel;

            if(fabs(_tw.linear.x) > 0.2 )
                _tw.linear.x = _SIGN(_tw.linear.x)*0.2;
            if(fabs(_tw.linear.x) < 0.1 )
                _tw.linear.x = _SIGN(_tw.linear.x)*0.1;
            if(fabs(_tw.angular.z) > 0.3 )
                _tw.angular.z = _SIGN(_tw.angular.z)*0.3 ;
            velPub.publish(_tw);
            return;
        }

        raw.state = Ctrl_PLAN;

        p0 << 0.0, 0.0;
        p1 << endP.y()/tan(oParam.consAngle) , endP.y();
    //    p2 << endP.x() - endP.norm()/4.0*cos(cmd.targetAngle), endP.y() - endP.norm()/4.0* sin(cmd.targetAngle);
        p2 << endP.x() - endP.norm()/2.0*cos(hraw.targetAngle), endP.y() - endP.norm()/2.0* sin(hraw.targetAngle);
        p3 << endP.x(), endP.y();

        Vector2d vel2d, acc2d;
        vel2d = 3*(p1 - p0);
        acc2d = 6*(p0-2*p1+p2);

        double line_acc = 1.0/(2.0*vel2d.norm())*4*vel2d.dot( acc2d/2 );
        double omega = sqrt( acc2d.norm()*acc2d.norm() - line_acc*line_acc  )/vel2d.norm();

        Vector2d omega2d;
        omega2d = (acc2d - line_acc*vel2d.normalized()).normalized();
        double dir = atan2(omega2d.y(),omega2d.x());
        dir = angleMainValue(dir-oParam.consAngle);
        dir = dir>0? 1: -1;

        _tw.linear.x = bezier_param.klinear*vel2d.norm();
        if(_tw.linear.x < 0.35) _tw.linear.x = 0.35;
        if(_tw.linear.x > 1.0) _tw.linear.x = 1.0;
        _tw.angular.z = bezier_param.kangular*(dir)*omega;

        velPub.publish(_tw);

        mapMat = Mat::zeros(Size(800, 800), CV_8UC3);
        circle(mapMat, Point( p0.x()*oParam.coef*100+oParam.origin.x(), oParam.origin.y()-p0.y()*oParam.coef*100 ),10,Scalar(255,255,255),-1);
        circle(mapMat, Point( p1.x()*oParam.coef*100+oParam.origin.x(), oParam.origin.y()-p1.y()*oParam.coef*100 ),10,Scalar(255,255,0),-1);
        circle(mapMat, Point( p2.x()*oParam.coef*100+oParam.origin.x(), oParam.origin.y()-p2.y()*oParam.coef*100 ),10,Scalar(255,0,255),-1);
        circle(mapMat, Point( p3.x()*oParam.coef*100+oParam.origin.x(), oParam.origin.y()-p3.y()*oParam.coef*100 ),10,Scalar(0,0,255),-1);

        for (double t = 0; t <1; t+=0.01) {
            Vector2d pp;
            pp  = pow(1-t,3)*p0 + 3*t*pow(t-1,2)*p1 + 3*pow(t,2)*(1-t)*p2 + pow(t,3)*p3;
            Point loc = Point(pp.x()*100*oParam.coef+oParam.origin.x(),oParam.origin.y()-pp.y()*100*oParam.coef);
            circle(mapMat, loc,1,Scalar(255,0,0), -1 );
        }
        mapOK = true;
    }

    void odomCallback( const nav_msgs::Odometry::ConstPtr &odom ){
        raw.position.x() = -odom->pose.pose.position.y;
        raw.position.y() = odom->pose.pose.position.x;

        double yaw=tf::getYaw(odom->pose.pose.orientation); //
        raw.curAngle = yaw;

        static int first = 0;
        if(!first && hseg->getRaw().state == HSeg_OK ){
            first = 1;
            bezierTimer = n.createTimer(ros::Duration(0.05), &bezierController::bezierCtrlThread, this);
        }
    }

    void navParamCallback( const geometry_msgs::Point::ConstPtr &pid ){
        nav_param.kp_v = pid->x;
        nav_param.kp_a = pid->y;
        nav_param.kv_a = pid->z;
    }

    void bezierParamCallback( const geometry_msgs::Point::ConstPtr &pid ){
        bezier_param.klinear = pid->x;
        bezier_param.kangular = pid->y;
    }


public:
    bezierController( ros::NodeHandle &nh, HCharSeg *segPtr ){

        mapOK = false;

        bezier_param.klinear = 0.06;
        bezier_param.kangular = 0.55;

        nav_param.kp_v = 0.4;
        nav_param.kp_a = 0.0;
        nav_param.kv_a = 0.4;

        hseg = segPtr;

        n = nh;

        odomSub = n.subscribe("/odom", 10, &bezierController::odomCallback, this);
        navParamSub = n.subscribe("/pid_nav", 10, &bezierController::navParamCallback, this);
        bezierParamSub = n.subscribe("/bezier", 10, &bezierController::bezierParamCallback, this);

        velPub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 1);
        raw.state = Ctrl_PLAN;
    }

    void info(Mat frIn){
        if(mapOK){
            char linear[20], angular[20], distc[20];
            sprintf(linear, "linear=%0.2f", _tw.linear.x );//
            sprintf(angular, "Angle=%0.2f", _tw.angular.z);//
            sprintf(distc, "Dist=%0.2f", dist);//


            char *hRes, *runRes;

            switch (hseg->getRaw().state) {
                case HSeg_NONE:
                    hRes = "H_Seg: NONE";
                    break;
                case HSeg_OK:
                    hRes = "H_Seg: OK";
                    break;
            }


            switch (raw.state) {
                case dipLib::Ctrl_NAV:
                    runRes = "RUN: NAV";
                    break;
                case Ctrl_PLAN:
                    runRes = "RUN: Bezier";
                    break;
                case Ctrl_STOPPED:
                    runRes = "RUN: Stopped";
                    break;
            }


            rectangle(mapMat, Point( mapMat.cols - 200, 2 ), Point( mapMat.cols, 100 ),Scalar(0, 0, 0), -1 );
            putText(mapMat, hRes, Point( mapMat.cols - 200, 20 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
            putText(mapMat, runRes, Point( mapMat.cols - 200, 40 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
            putText(mapMat, linear, Point( mapMat.cols - 200, 60 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
            putText(mapMat, angular, Point( mapMat.cols - 200, 80 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
            putText(mapMat, distc, Point( mapMat.cols - 200, 100 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));


            imshow("Map", frIn+mapMat);
        }
    }

};

int main(int argc, char ** argv){
    ros::init(argc, argv, "dip_proj");
    ros::NodeHandle n("~");

    VideoCapture cp;
    while(!cp.isOpened()){
        static int tt = 2;
        cp.open(tt++);
    }

    Mat frIn;
    for (int i = 0; i < 30; ++i) {
        cp.read(frIn);
    }


    Mat perspectiveMat;
    oParam.coef = 2;
#if 0
//    标定透视变换矩阵
    /* 左上 右上 右下 左下 地砖算100*coef像素 */

    Point2f corner[] = { Point2f(261,109),
                         Point2f(379,110),
                         Point2f(397,143),
                         Point2f(237,144)  };
    Point2f canvas[] = { Point2f(oParam.coef*120,oParam.coef*223),
                         Point2f(oParam.coef*180,oParam.coef*223),
                         Point2f(oParam.coef*180,oParam.coef*280),
                         Point2f(oParam.coef*120,oParam.coef*280) };

    Mat M = getPerspectiveTransform(corner, canvas);

    for (int i = 0; i < M.rows; ++i) {
        for (int j = 0; j < M.cols; ++j) {
            cout << M.at<double>(i,j) << ",";
        }
        cout << endl;
    }

    Mat pp;
    while(ros::ok()){
        cp.read(frIn);
        frIn = frIn(cv::Rect(0, 0, frIn.cols / 2, frIn.rows));
        warpPerspective(frIn, pp, M, Size(800, 800));
        imshow("src", frIn);
        imshow("pp", pp);
        waitKey(5);
    }

#endif

    perspectiveMat =(Mat_<double>(3,3)<<
29.8021,109.833,-11221.6,
-9.39043,317.899,-16351.2,
-0.018066,0.3601,1
);

    /* 根据黄线标定车的朝向 */
    oParam.origin.x() = 309.0;
    oParam.origin.y() = 764.0;
    double consK = (oParam.origin.y()-175.0)/(oParam.origin.x()-320.0);
    oParam.consAngle = angleMainValue(pi - angleMainValue(atan(consK)));
    while(oParam.consAngle<0) oParam.consAngle += pi;


    HCharSeg hSeg;
    bezierController bCtrl(n, &hSeg);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    while (ros::ok()){
        cp.read(frIn);

        if(frIn.empty()) continue;

        frIn = frIn(cv::Rect(0, 0, frIn.cols / 2, frIn.rows));
        imshow("PC Camera",frIn);

        Mat ppMat;
        warpPerspective(frIn, ppMat, perspectiveMat, Size(800, 800));

        hSeg.load(ppMat);

        bCtrl.info(ppMat);

        line(frIn, Point(frIn.cols/2, 0), Point(frIn.cols/2, frIn.rows),Scalar(0,255,255));
        warpPerspective(frIn, ppMat, perspectiveMat, Size(800, 800));
        imshow("perspective", ppMat);

        waitKey(5);

    }



}
