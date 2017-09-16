package org.firstinspires.ftc.teamcode;

import android.media.Image;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.ColorBlobDetector;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.ColorHSV;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
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

public class Vision  {

    Runnable run = new Runnable() {
        //@Override
        public void run() {

        }
    };


    void loop() {
        VideoCapture capture = new VideoCapture();
        Mat frame = new Mat();
        ScheduledExecutorService timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(run, 0, 33, TimeUnit.MILLISECONDS);
        MatOfByte buffer = new MatOfByte();
        Imgcodecs.imencode(".png", frame, buffer);








    }

}
