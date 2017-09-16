package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


/**
 * Created by pranav on 9/15/17.
 */

public class VisionAnton {

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
