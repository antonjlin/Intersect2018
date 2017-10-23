package org.firstinspires.ftc.teamcode.robotutil;

import android.app.Activity;
import android.graphics.Bitmap;
import android.view.View;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.opencv.android.JavaCameraView;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by pranav on 10/22/17.
 */

public class BallDetector {

    private  int redPositionX;
    private int redPositionY;
    Mat rgba;
    Mat grey;




    public BallDetector(Mat rgba, Mat grey){
        this.rgba = rgba;
        this.grey = grey;
    }

    public void detectPosition(Mat rgba) {


        this.redPositionX = 0;//
        this.redPositionY = 0;//
    }

    public int[] getRebBallPosition() {

        return new int[]{redPositionX, redPositionY};
    }
}
