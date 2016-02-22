#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
//#include "csvparser/csvparser.h"

#define X_ROI 10
#define Y_ROI 10
#define WIDTH 40
#define HEIGHT 40

float fiducial[3][2] = {
        {172, 447} ,
        {512, 446} ,
        {503, 14}
};



float data[3][2] = {0};


int locateFiducial(float target_x, float target_y, float* fiducial_x, float* fiducial_y, float* fiducial_r);
void rigidTransform(cv::Mat A, cv::Mat B, cv::Mat& R, cv::Mat& t);

using namespace cv;
using namespace std;

int main()
{
    int fidNum, i;
    fidNum = sizeof(fiducial)/sizeof(fiducial[0]);
    for (i = 0; i<fidNum; i++)
    {
        float fid_x, fid_y, fid_r;
        locateFiducial(fiducial[i][0],fiducial[i][1],&fid_x,&fid_y,&fid_r);
        data[i][0]= fid_x;
        data[i][1]= fid_y;
        cout << "Fiducial #" << i+1 << ": x:" << fid_x << " y:" << fid_y <<  "\n======================\n";
    }

    Mat A, B, R, t;
    A = Mat(3, 2, CV_32F, fiducial);
    B = Mat(3, 2, CV_32F, data);
    rigidTransform(A,B,R,t);

    cout << "R = "<< endl << " "  << R << endl << endl;
    cout << "t = "<< endl << " "  << t << endl << endl;





    //waitKey();
    return 0;

}

int locateFiducial(float target_x, float target_y, float* fiducial_x, float* fiducial_y, float* fiducial_r)
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
            float x_roi = target_x - X_ROI, y_roi = target_y - Y_ROI;

            // Uncomment for saving the frame to disk
            //imwrite( "/home/vassilis/Dropbox/Uppsala Universitet/Thesis/Selective-soldering-PC/opencv_board.jpg", src );

            // Create the ROI and crop the frame
            Rect region_of_interest = Rect((int)x_roi,(int) y_roi, WIDTH, HEIGHT);
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
                circle(src, Point((int)c[0] + (int)x_roi, (int)c[1] + (int)y_roi), c[2], Scalar(0, 0, 255), 3, LINE_AA); // +x_roi and +y_roi for displaying using the whole's frame
                circle(src, Point((int)c[0] + (int)x_roi, (int)c[1] + (int)y_roi), 2, Scalar(0, 255, 0), 3, LINE_AA);   // coordinates rather than the ROI's ones (local)
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

/*  NAME:               void rigidTransform(cv::Mat A, cv::Mat B, cv::Mat& R, cv::Mat& t)
 *
 *  DESCRIPTION:        This function finds the optimal Rigid/Euclidean transform
 *
 *  INPUTS:
 *  -cv::Mat A          The fiducial coordinates read from the BOM file
 *
 *
 *  -cv::Mat B          The current fiducial coordinates
 *
 *
 *  OUTPUTS:
 *  -cv::Mat& R         Optimal rotation matrix
 *
 *  -cv::Mat& t         Translation matrix
 */

void rigidTransform(cv::Mat A, cv::Mat B, cv::Mat& R, cv::Mat& t)
{
    int N; // Number of points
    Mat S,U,V; // SVD matrices
    Mat centroid_A, centroid_APrime,centroid_BPrime, centroid_B, H, temp, trans;


    reduce(A,centroid_A, 0, CV_REDUCE_AVG); // MATLAB: centroid_A = mean(A);
    reduce(B,centroid_B, 0, CV_REDUCE_AVG); // MATLAB: centroid_B = mean(B);

    N = A.rows;                             // MATLAB: N = size(A,1);

    // MATLAB:  H = (A - repmat(centroid_A, N, 1))' * (B - repmat(centroid_B, N, 1));
    temp = A - repeat(centroid_A, N, 1); // A - repmat(centroid_A, N, 1)
    transpose(temp, trans); // (A - repmat(centroid_A, N, 1))'
    H = trans * (B - repeat(centroid_B, N, 1)); //   H = (A - repmat(centroid_A, N, 1))' * (B - repmat(centroid_B, N, 1));

    // MATLAB: [U,S,V] = svd(H);
    cv::SVD s; // SVD constructor
    s.compute(H,S,U,V); // Compute SVD

    // We get S as a single column, put the values at a diagonal to match MATLAB's results
    //S = Mat::diag(S); // Commented out as it is not (currently) used

    // Negate U(:,1) to match MATLAB's results
    temp = U.col(0);
    U.col(0) = temp * -1;

    // Negate V(1,:) to match MATLAB's results
    temp = V.row(0);
    V.row(0) = temp * -1;

    // MATLAB: R = V*U';
    transpose(U, trans); // U'
    R = V * trans; // R = V*U';

    if(determinant(R)<0)
    {
        cout << "Reflection detected" << endl;
        temp = V.col(1); // MATLAB: temp = V(:,2);
        V.col(1) = temp * -1; // MATLAB: V(:,2) = temp * -1;
        R = V * trans; // MATLAB: R = V*U';
    }
    //cout << "R = "<< endl << " "  << R << endl << endl;

    // MATLAB: t = -R*centroid_A' + centroid_B'
    transpose(centroid_A, centroid_APrime); // centroid_A'
    transpose(centroid_B, centroid_BPrime); // centroid_B'
    t = -R * centroid_APrime + centroid_BPrime; // t = -R*centroid_A' + centroid_B'
    //cout << "t = "<< endl << " "  << t << endl << endl;





    float data3[1][2] = {32, 12};
    Mat test, test_tr, A2;
    test = Mat(1, 2, CV_32F, data3);

    //A2 = (ret_R*A') + repmat(ret_t, 1, n);
    transpose(A, trans);
    A2 = (R*trans) + repeat(t, 1, N);
    //test_tr = (ret_R*test') + repmat(ret_t, 1, 1);
    transpose(test,trans);
    test_tr = (R * trans) + repeat(t, 1, 1);
    transpose(test_tr,trans);
    cout << "test_tr = "<< endl << " "  << trans << endl << endl;

    transpose(A2, trans);
    cout << "A2 = "<< endl << " "  << trans << endl << endl;
}