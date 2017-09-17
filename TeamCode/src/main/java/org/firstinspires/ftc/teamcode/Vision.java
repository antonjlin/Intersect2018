package org.firstinspires.ftc.teamcode;

import android.media.Image;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebView;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceManager;

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
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Algorithm;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;


import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static android.R.id.input;
import static org.lasarobotics.vision.opmode.VisionOpMode.rotation;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.CV_CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.CV_RETR_LIST;
import static org.opencv.imgproc.Imgproc.RETR_CCOMP;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.*;


/**
 * Created by pranav on 9/15/17.
 */

public class Vision extends OpMode {

    Runnable run = new Runnable() {
        //@Override
        public void run() {
        }
    };

    ImageView view  = (ImageView) View.findViewById(R.id.MyImageDisplay);

    public void init(){

        ImageView view = new ImageView();
    }



    public void loop() {

        VideoCapture capture = new VideoCapture(0);
        Mat frame = new Mat();
        Mat greyframe = new Mat();
        capture.read(frame);
        telemetry.addData();


        List<MatOfPoint> countours = new ArrayList<MatOfPoint>(){};
        Mat Hierarchy = new Mat();

        findContours(greyframe,countours,Hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        //drawContours();





        /*ColorHSV min = new ColorHSV(255,50,80);
        ColorHSV max = new ColorHSV(205,100,100);
        ColorBlobDetector detector = new ColorBlobDetector(min, max);
        System.out.print(detector.getContours());
        telemetry.addData("getCountours",detector.getContours());*/
        Imgproc.cvtColor(frame, greyframe, Imgproc.COLOR_RGB2GRAY);



        ScheduledExecutorService timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(run, 0, 33, TimeUnit.MILLISECONDS);


    }
    public void runOpMode(){

    }

}
