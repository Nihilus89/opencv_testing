#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#define X_ROI 10
#define Y_ROI 10
#define WIDTH 40
#define HEIGHT 40

int fiducial[3][2] = {
        {172, 447} ,
        {512, 446} ,
        {503, 14}
};


int locateFiducial(int target_x, int target_y, int* fiducial_x, int* fiducial_y, int* fiducial_r);

using namespace cv;
using namespace std;

int main()
{
    int fidNum, i;
    fidNum = sizeof(fiducial)/sizeof(fiducial[0]);
    for (i = 0; i<fidNum; i++)
    {
        int fid_x, fid_y, fid_r;
        locateFiducial(fiducial[i][0],fiducial[i][1],&fid_x,&fid_y,&fid_r);
        cout << "Fiducial #" << i+1 << ": x:" << fid_x << " y:" << fid_y <<  "\n======================\n";
    }
    //waitKey();
    return 0;

}

int locateFiducial(int target_x, int target_y, int* fiducial_x, int* fiducial_y, int* fiducial_r)
{
    VideoCapture stream(0);

    if(!stream.isOpened())
        cout << "\nCannot open video camera";
    else
    {
        while (true)
        {
            // Disable autofocus and set a manual value
            system("v4l2-ctl -d 0 -c focus_auto=0");
            system("v4l2-ctl -d 0 -c focus_absolute=30");

            // Capture a frame from the stream
            Mat src;
            stream.read(src);

            // The x,y coordinates of the ROI's starting point
            int x_roi = target_x - X_ROI, y_roi = target_y - Y_ROI;

            // Uncomment for saving the frame to disk
            //imwrite( "/home/vassilis/Dropbox/Uppsala Universitet/Thesis/Selective-soldering-PC/opencv_board.jpg", src );

            // Create the ROI and crop the frame
            Rect region_of_interest = Rect(x_roi, y_roi, WIDTH, HEIGHT);
            Mat roi = src(region_of_interest);

            // Convert to grayscale and perform edge detection
            Mat dst, cdst;
            Canny(roi, dst, 50, 200, 3);
            cvtColor(dst, cdst, CV_GRAY2BGR);

            // Perform the circles Hough transform
            vector<Vec3f> circles;
            HoughCircles(dst, circles, HOUGH_GRADIENT, 1, 100, 100, 10, 3, 5); //image, output vector, method, dp, minDist, param1, param2, minRadius, maxRadius
            // Display the circles
            for (size_t i = 0; i < circles.size(); i++) {
                Vec3i c = circles[i];
                circle(src, Point(c[0] + x_roi, c[1] + y_roi), c[2], Scalar(0, 0, 255), 3, LINE_AA); // +x_roi and +y_roi for displaying using the whole's frame
                circle(src, Point(c[0] + x_roi, c[1] + y_roi), 2, Scalar(0, 255, 0), 3, LINE_AA);   // coordinates rather than the ROI's ones (local)
                // Return values
                *fiducial_x = c[0] + x_roi;
                *fiducial_y = c[1] + y_roi;
                *fiducial_r = c[2];
            }

            // Appending on the frame a rectangle to visualize the ROI
            Mat color(roi.size(), CV_8UC3, Scalar(0, 200, 200));
            double alpha = 0.5; // transparency
            addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);

            // Show the image
            imshow("source", src);
            //Uncomment to show the ROI in edge
            //imshow("detected circles", cdst);

            // Timeouted wait for ESC press or a single circle (the fiducial) has been detected
            if(waitKey(10) == 27 || circles.size() == 1)
                break;
        }
    }
    return 0;
}
