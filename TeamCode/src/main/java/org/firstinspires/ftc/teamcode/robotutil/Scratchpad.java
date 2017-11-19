package org.firstinspires.ftc.teamcode.robotutil;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import static org.opencv.imgproc.Imgproc.CV_HOUGH_GRADIENT;
import static org.opencv.imgproc.Imgproc.HOUGH_GRADIENT;
import static org.opencv.imgproc.Imgproc.bilateralFilter;

/**
 * Created by pranav on 10/29/17.
 */

public class Scratchpad {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

        public static void main(String[] args) {
            Mat src = Imgcodecs.imread(("/home/pranav/Desktop/jewelImage.jpg"));
            findCircles(src);

        }

    public static void findCircles(Mat src) {
//        image    8-bit, single-channel, grayscale input image.
//        circles  Output Mat of found circles. Each row is encoded as a 3-element floating-point vector \((x, y, radius)\) .
//        method   Detection method, see cv::HoughModes. Currently, the only implemented method is HOUGH_GRADIENT
//        dp   Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
//        minDist  Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
//        param1   First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny edge detector (the lower one is twice smaller).
//        param2   Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
//        minRadius    Minimum circle radius.
//        maxRadius    Maximum circle radius.
        int min = 20;
        int max = 200;
        Mat src_gray = new Mat();
        Mat bilateral = new Mat();
        Imgproc.cvtColor( src, src_gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(src_gray, src_gray, new Size(9, 9), 2, 2);
        Imgcodecs.imwrite("/tmp/blurred.png", src_gray);
        Mat circles = new Mat();
        bilateralFilter(src_gray, bilateral, 20000, 20000, 20000);

        /// Apply the Hough Transform to find the circles
        System.out.println("src_gray of rows a  nd cols "  + src_gray.cols() + " " + src_gray.rows());
        Imgproc.HoughCircles( bilateral, circles, CV_HOUGH_GRADIENT, 1, bilateral.rows()/20 );
        Imgproc.HoughCircles(bilateral,circles, HOUGH_GRADIENT, 1, bilateral.rows()/20, 200, 1, max, min);


        System.out.println("number of rows and cols "  + circles.cols() + " " + circles.rows());

        double x = 0.0;
        double y = 0.0;
        int r = 0;

        for( int i = 0; i < circles.rows(); i++ )
        {
            double[] data = circles.get(i, 0);
            for(int j = 0 ; j < data.length ; j++){
                x = data[0];
                y = data[1];
                r = (int) data[2];
            }
            Point center = new Point(x,y);
            // circle center

            // Imgproc.circle( src, center, 3, new Scalar(0,255,0), 4);
            // circle outline
            Imgproc.circle( src, center, r, new Scalar(0,0,255), 5      );
        }

        /// Draw the circles detected
//        for( size_t i = 0; i < circles.size(); i++ )
//        {
//            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//            int radius = cvRound(circles[i][2]);
//            // circle center
//            circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
//            // circle outline
//            circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
//        }

        Imgcodecs.imwrite("/tmp/withmarkers.jpg", src);

    }
}
