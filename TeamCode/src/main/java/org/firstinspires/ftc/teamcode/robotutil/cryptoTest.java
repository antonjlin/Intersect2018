package org.firstinspires.ftc.teamcode.robotutil;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_COLOR;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Created by pranav on 1/9/18.
 */

public class cryptoTest {


    public static void main(String[] args) {
        System.load("/Users/pranav/StudioProjects/Intersect2018/opencv-java");

        Mat RGB = Imgcodecs.imread("/Users/pranav/Downloads/Crypto.jpg",CV_LOAD_IMAGE_COLOR);



    }
}
