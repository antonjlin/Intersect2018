package org.firstinspires.ftc.teamcode;

import android.media.Image;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.ColorBlobDetector;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.Color;
import org.lasarobotics.vision.util.color.ColorHSV;
import org.lasarobotics.vision.util.color.ColorSpace;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Algorithm;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;


import java.io.ByteArrayInputStream;
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


/**
 * Created by pranav on 9/15/17.
 */

public class Vision extends OpMode {

    Runnable run = new Runnable() {
        //@Override
        public void run() {

        }
    };

    public void init(){

    }
    public void loop() {
        VideoCapture capture = new VideoCapture();
        Mat frame = new Mat();
        capture.read(frame);



        ColorHSV min = new ColorHSV(255,50,80);
        ColorHSV max = new ColorHSV(205,100,100);
        ColorBlobDetector detector = new ColorBlobDetector(min, max);
        System.out.print(detector.getContours());
        telemetry.addData("getCountours",detector.getContours());
       // telemetry.addData("")



        ScheduledExecutorService timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(run, 0, 33, TimeUnit.MILLISECONDS);
        MatOfByte buffer = new MatOfByte();
        Imgcodecs.imencode(".png", frame, buffer);

    }
    public void runOpMode(){

    }

}
