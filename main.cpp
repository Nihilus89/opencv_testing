#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture stream(0);

    if(!stream.isOpened()){
        cout << "\nCannot open video camera";
    } else {

        //CAPTURING FRAMES FROM CAMERA
        while( true ){
            Mat src;
            stream.read(src);

            //Mat src = imread("D:/LineDetection.png", 0);
            Rect region_of_interest = Rect(100, 200, 200, 200);
            Mat roi = src(region_of_interest);

            Mat dst, cdst;
            Canny(roi, dst, 50, 200, 3);
            cvtColor(dst, cdst, CV_GRAY2BGR);

            vector<Vec3f> circles;
            HoughCircles(dst, circles, HOUGH_GRADIENT, 1, 100,
                         100, 10, 3, 5); // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
            cout << "\nDetected " << circles.size() << " circles\n";
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Vec3i c = circles[i];
                circle( cdst, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
                circle( cdst, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
                cout << i << ":"<< " x,y:" << Point(c[0], c[1]) << " r:"<< c[2] << "\n";
            }
            cout << "\n--------------------------\n";
            Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 125));
            double alpha = 0.3;
            addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);

            imshow("source", src);
            imshow("detected circles", cdst);

            if( waitKey(10) == 27 )
                break;

        }

    }

    return 0;
}