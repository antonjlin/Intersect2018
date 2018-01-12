package org.firstinspires.ftc.teamcode.robotutil;

import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.tests.Vision;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import static java.lang.Math.PI;
import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.HoughLines;

/**
 * Created by pranav on 10/22/17.
 */

public class VisionAlign extends VisionProcessor{
    OpMode opMode;

    public void camera(){
        Activity activity = (Activity) opMode.hardwareMap.appContext;
        VisionProcessor visionProcessor = new VisionProcessor(activity);
        CameraBridgeViewBase.CvCameraViewListener2 cvCameraViewListener2 = new VisionProcessor(activity);


    }



    public static void lineDetect(){
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat res = new Mat();
        Mat original = Imgcodecs.imread("/home/pranav/Desktop/Crypto.jpg");
        Imgproc.cvtColor(original,hsv, Imgproc.COLOR_BGR2HSV);
        Scalar lower_red = new Scalar(0,1,1);
        Scalar upper_red = new Scalar(10,255,255);
        inRange(hsv, lower_red, upper_red,mask);
        bitwise_and(original,original,res ,mask );
        Imgcodecs.imwrite("/tmp/frame.jpg", original);
        Imgcodecs.imwrite("/tmp/mask.jpg", mask);
        Imgcodecs.imwrite("/tmp/res.jpg", res);
        Mat source = new Mat();
        Imgproc.cvtColor( original, source, Imgproc.COLOR_BGR2GRAY);
        Imgcodecs.imwrite("/tmp/crypto.jpg", source);
        Imgcodecs.imwrite("/tmp/blur.jpg", source);
        Mat vector = new Mat();
        HoughLines(mask, vector, 3, PI/5, 200, 1, 1,0, PI/9);
        //HoughLinesP(mask, vector, 1, PI/180, 80, 30, 10);
        for (int i = 0; i < vector.rows(); i++) {
            double data[] = vector.get(i,0 );
            double rho1 = data[0];
            System.out.println(rho1);
            double theta1 = data[1];
            double cosTheta = Math.cos(theta1);
            double sinTheta = Math.sin(theta1);
            double x0 = cosTheta * rho1;
            double y0 = sinTheta * rho1;
            Point pt1 = new Point(x0 + 10000 * (-sinTheta), y0 + 10000 * cosTheta);
            Point pt2 = new Point(x0 - 10000 * (-sinTheta), y0 - 10000 * cosTheta);
            Imgproc.line(original, pt1, pt2, new Scalar(0, 0, 255), 2);
        }
        System.out.println("rows " + vector.rows() + "  "+ "columns " + vector.cols() );
        Imgcodecs.imwrite("/tmp/withLines.jpg", original);
/*
           Imgproc.threshold(source, source, 127, 255, THRESH_BINARY);
           Mat skel = new Mat(source.size(), CV_8UC1, new Scalar(0));
           Mat temp = new Mat(source.size(), CV_8UC1);
           Mat element = getStructuringElement(MORPH_CROSS, new Size(3, 3));
           boolean done;
           do{
               morphologyEx(source, temp, MORPH_OPEN, element);
               bitwise_not(temp, temp);
               bitwise_and(source, temp, temp);
               bitwise_or(skel, temp, skel);
               erode(source, source, element);
               double max;
               MinMaxLocResult minMx = minMaxLoc(source);
               System.out.println(minMx.maxLoc);
               done = (minMx.maxVal == 0 );
           }while(!done);
           Imgcodecs.imwrite("/tmp/skel.jpg", skel);*/
    }

    public void init(OpMode opMode){
        this.opMode = opMode;
    }


}
