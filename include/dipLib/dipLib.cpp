//
// Created by stdcat on 12/2/21.
//

#include "dipLib.h"

using namespace dipLib;

/* dip Color Segment */

Mat dipColorSegment::segment(Mat bgr) {

    Mat hsv;
    cvtColor(bgr, hsv, CV_BGR2HSV);
    Mat bin = Mat::zeros( bgr.size(), CV_8UC1 );

    for (int i = 0; i < bin.rows; ++i) {
        for (int j = 0; j < bin.cols; ++j) {

            int h = hsv.at<Vec3b>(i,j)[0];
            int s = hsv.at<Vec3b>(i,j)[1];
            int v = hsv.at<Vec3b>(i,j)[2];

            if(hMin > hMax){
                if(
                    sMin <= s && s <= sMax &&
                    vMin <= v && v <= vMax
                ){
                    if(  h >= hMin || h <= hMax  )
                        bin.at<uchar>(i,j) = 255;

                }
            }
            else{
                if(
                    hMin <= h && h <= hMax &&
                    sMin <= s && s <= sMax &&
                    vMin <= v && v <= vMax
                ){
                    bin.at<uchar>(i,j) = 255;
                }
            }



        }
    }

    return bin;
}

dipColorSegment::dipColorSegment(
            int _hMin,
            int _hMax,
            int _sMin,
            int _sMax,
            int _vMin,
            int _vMax
            ):
            hMin(_hMin),
            hMax(_hMax),
            sMin(_sMin),
            sMax(_sMax),
            vMin(_vMin),
            vMax(_vMax)
{}

void dipColorSegment::setSegParam(int _hMin,
                                         int _hMax,
                                         int _sMin,
                                         int _sMax,
                                         int _vMin,
                                         int _vMax)
{
    hMin = _hMin;
    hMax = _hMax;
    sMin = _sMin;
    sMax = _sMax;
    vMin = _vMin;
    vMax = _vMax;
}

void dipColorSegment::setSegParam(VectorXi param) {
    if( param.size() != 6 ) return;
    hMin = param(0);
    hMax = param(1);
    sMin = param(2);
    sMax = param(3);
    vMin = param(4);
    vMax = param(5);
}



/* H Char Segment */

HCharSeg::HCharSeg():
    dipColorSegment(    64,
                    129,
                    24,
                    255,
                    109,
                    255),
    gammaPercent(57)
{
    namedWindow("Color_Segment");
    createTrackbar("H_min", "Color_Segment", &this->hMin, 180);
    createTrackbar("H_max", "Color_Segment", &this->hMax, 180);
    createTrackbar("S_min", "Color_Segment", &this->sMin, 255);
    createTrackbar("S_max", "Color_Segment", &this->sMax, 255);
    createTrackbar("V_min", "Color_Segment", &this->vMin, 255);
    createTrackbar("V_max", "Color_Segment", &this->vMax, 255);
    createTrackbar("gamma", "Color_Segment", &this->gammaPercent, 100);

}

void HCharSeg::GammaCorrection(Mat src, Mat &dst, float fGamma)
{
    // build look up table
    unsigned char lut[256];
    for( int i = 0; i < 256; i++ )
    {
        lut[i] = saturate_cast<uchar>(pow((float)(i/255.0), fGamma) * 255.0f);//防止颜色溢出操作
    }
    dst = src.clone();
    const int channels = dst.channels();
    switch(channels)
    {
        case 1:
            {
                MatIterator_<uchar> it, end;
                for( it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++ )
                    *it = lut[(*it)];
                break;
            }
        case 3:
            {

                MatIterator_<Vec3b> it, end;
                for( it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++ )
                {
                    (*it)[0] = lut[((*it)[0])];
                    (*it)[1] = lut[((*it)[1])];
                    (*it)[2] = lut[((*it)[2])];
                }
                break;
            }
    }
}

Vector2d HCharSeg::getCenter(vector<Point> li) {
    Vector2d pt;
    double x = 0, y = 0;
    for (auto it:li) {
        x += it.x;
        y += it.y;
    }
    x /= li.size();
    y /= li.size();
    pt.x() = x;
    pt.y() = y;
    return pt;
}

void HCharSeg::load(Mat src) {

    /* 1. Gamma Correction */
    GammaCorrection(src, src, float((float)gammaPercent/100.0f));

    /* 2. Color Segment */
    Mat bin = segment(src);
    cvtColor(bin, bin, CV_GRAY2BGR);
    imshow("Color_Segment", bin&src);
    cvtColor(bin, bin, CV_BGR2GRAY);

    /* 3. select Biggest Contours */
    vector<vector<Point>> contours; //定义轮廓集合
    vector<Vec4i> hierarchy;

    findContours(bin, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//CV_RETR_EXTERNAL只检测外部轮廓

    /* if nothing, return */
    if(contours.empty()){
        raw.state = HSeg_NONE;
        return;
    }

    // draw black contours on white image
    Mat hArea(bin.size(),CV_8UC3,Scalar(0,0,0));
    int index = 0;
    vector<pair<Rect, int > > rectList;// 存放矩形框和轮廓索引
    for (; index >= 0; index = hierarchy[index][0]) //hierarchy[index][0]表示后一个轮廓
    {

        Rect rect = boundingRect(contours[index]);//检测外轮廓
        if(rect.width*rect.height > 100 )// 过滤小噪声
            rectList.push_back(make_pair(rect, index));
    }

    std::sort(rectList.begin(), rectList.end(), rectCMP);

    for (auto it:rectList) {
        if( (fabs( (it.first.x+it.first.width/2) - (rectList[0].first.x + rectList[0].first.width/2) ) < rectList[0].first.width &&
            fabs( (it.first.y+it.first.height/2) - (rectList[0].first.y + rectList[0].first.height/2) ) < rectList[0].first.height)||
            it.first.height*it.first.height > 500
        ){
            drawContours(hArea, contours, it.second, Scalar(255,255,255), -1, 8);//描绘字符的外轮廓
        }

    }

    /* 4. open & close */
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(hArea, hArea, MORPH_OPEN, element);
    //闭操作 (连接一些连通域)
    morphologyEx(hArea, hArea, MORPH_CLOSE, element);

#define USE_MIN_RECT 0
#if USE_MIN_RECT
    /* 最小外接矩形法使用的副本 */
    Mat hRect = hArea.clone();
    cvtColor(hRect,hRect, CV_BGR2GRAY);
#endif

#define USE_CONVEX_HULL 1
#if USE_CONVEX_HULL

    /* 5. convex hull */
    contours.clear();
    hierarchy.clear();
    cvtColor(hArea, hArea, CV_BGR2GRAY);
    findContours(hArea, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//CV_RETR_EXTERNAL只检测外部轮廓
    vector<vector<Point>> convex(contours.size());//凸包轮廓点
    for (size_t i = 0; i < contours.size(); i++)
    {
        convexHull(contours[i], convex[i], false, true);
    }
    Mat convexMat = Mat::zeros(hArea.size(), CV_8UC1 );
    Vector2d convexCenter;
    for (size_t k = 0; k < contours.size(); k++)
    {
        static double lastArea = 0;
        vector<Vec4i> empty(0);
        //绘制凸包点
        drawContours(convexMat, convex, (int)k, Scalar (255), -1, LINE_AA, empty, 0, Point(0, 0));
        if(contourArea(convex[(int)k]) > lastArea) convexCenter = getCenter(convex[(int)k]);
    }


    /* 6. subtract & open */
    Mat deltaMat = convexMat - hArea;
    morphologyEx(deltaMat, deltaMat, MORPH_OPEN, element);

    /* 7. only 2 areas & get dir */
    contours.clear();
    hierarchy.clear();
    findContours(deltaMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//CV_RETR_EXTERNAL只检测外部轮廓
    vector<pair<int, int > > deltaList;
    for (index = 0; index < contours.size(); index ++) //hierarchy[index][0]表示后一个轮廓
    {
        int area =  contourArea(contours[index]);
        deltaList.push_back(make_pair(area, index));

    }
    sort(deltaList.begin(), deltaList.end(), areaCMP);
    deltaMat = Mat::zeros(deltaMat.size(), CV_8UC1);
    for (int i = 0; i < 2 && i< deltaList.size(); ++i) {
        drawContours(deltaMat, contours, deltaList[i].second, Scalar(255), -1, 8, hierarchy);
    }


    Vector2d p1, p2;//H凹陷的2个区域中心点
    Vector2d centerPoint;
    double centerAngle;

    if(deltaList.size() >= 2){
        cvtColor(deltaMat, deltaMat, CV_GRAY2BGR);

        p1 = getCenter(contours[deltaList[0].second]);
        p2 = getCenter(contours[deltaList[1].second]);

        centerPoint.x() = (p1.x() + p2.x())/2;
        centerPoint.y() = (p1.y() + p2.y())/2;
        centerAngle = angleMainValue(pi - atan2(p1.y() - p2.y(), p1.x()-p2.x() ) );

        while( centerAngle < 0 ) centerAngle += pi;

        raw.targetAngle = centerAngle;
        raw.targetPoint = centerPoint;
        raw.state = HSeg_OK;

        line(deltaMat, Point(p1.x(), p1.y()), Point(p2.x(), p2.y()), Scalar(255,255,0), 1, 8);
    }
    else if(deltaList.size() == 1){
        p1 = getCenter(contours[deltaList[0].second]);
        centerPoint = p1;

        /* keep targetPoint */
        raw.targetPoint = centerPoint;
        raw.state = HSeg_OK;
    }
    else{
        raw.targetPoint = centerPoint;
        raw.state = HSeg_OK;
    }

    circle(deltaMat, Point(raw.targetPoint.x(), raw.targetPoint.y()),10,Scalar(255,0,255),-1);
    imshow("delta", deltaMat);

#endif

#if USE_MIN_RECT
    contours.clear();
    findContours(hRect, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    RotatedRect box;
    Rect boundRect;
    Mat hMat = Mat::zeros(hRect.size(), CV_8UC3);
    Point2f rect_point[4];

    for (auto itc:contours) {
        box = minAreaRect(Mat(itc));  //计算每个轮廓最小外接矩形(旋转)
        boundRect = box.boundingRect();
        //boundRect = boundingRect(Mat(*itc));
        circle(hMat, Point(box.center.x, box.center.y), 5, Scalar(255,0, 0), -1, 8);  //绘制最小外接矩形的中心点
        // rectangle(dstImg, Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), Scalar(0, 255, 0), 2, 8);
        rectangle(hMat, boundRect.tl(), boundRect.br() , Scalar(0, 255, 0), 1, 8);
        box.points(rect_point);  //把最小外接矩形四个端点复制给rect数组
        for (int j = 0; j<4; j++)
         {
        line(hMat, rect_point[j], rect_point[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
         }
        cout << "angle " <<box.angle << endl;
        cout << "width " <<box.size.width << endl;
        cout << "height "<<box.size.height << endl<<endl;
        char width[20], height[20];
        sprintf(width, "center=%0.2f %0.2f", box.center.x, box.center.y );//
        sprintf(height, "Angle=%0.2f", box.angle);//
        putText(hMat, width, box.center, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
        putText(hMat, height, box.center + Point2f(0, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
    }
    imshow("hmat",hMat);
#endif

}

HSeg_RAW HCharSeg::getRaw() {
    if(raw.state == HSeg_NONE){
        HSeg_RAW ret;
        ret.state = HSeg_NONE;
        return ret;
    }
    if(raw.state == HSeg_OK){
        return raw;
    }
}


