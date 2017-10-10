package org.firstinspires.ftc.teamcode.robotutil;

import android.app.Activity;
import android.graphics.Bitmap;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;


import com.qualcomm.robotcore.eventloop.opmode.*;

import org.lasarobotics.vision.android.Sensors;
//import org.lasarobotics.vision.opmode.VisionOpModeCore;
import org.lasarobotics.vision.util.FPS;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;


import static com.sun.tools.javac.util.Assert.error;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;


/**
 * Created by pranav on 9/15/17.
 */

//@SuppressWarnings("ALL")

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision1")

//@com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar("vision", Vision)
//@com.qualcomm.robotcore.eventloop.opmode.TeleOpNew\


public class Vision extends OpMode {
    private static final int initialMaxSize = 1200;
    public static JavaCameraView openCVCamera;
    private static boolean initialized = false;
    private static boolean openCVInitialized = false;
    public int width, height;
    public FPS fps;
    public Sensors sensors;

    Runnable run = new Runnable() {
        //@Override
        public void run() {
        }
    };


   // ImageView view  = (ImageView) View.findViewById(R.id.MyImageDisplay);

    public void init(){
        BaseLoaderCallback openCVLoaderCallback = null;
        try {
            openCVLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
                @Override
                public void onManagerConnected(int status) {
                    switch (status) {
                        case LoaderCallbackInterface.SUCCESS: {
                            //Woohoo!
                            Log.d("OpenCV", "OpenCV Manager connected!");
                            openCVInitialized = true;
                        }
                        break;
                        default: {
                            super.onManagerConnected(status);
                        }
                        break;
                    }
                }
            };
        } catch (NullPointerException e) {
            error("Could not find OpenCV Manager!\r\n" +
                    "Please install the app from the Google Play Store.");
        }

        final Activity
                activity = (Activity) hardwareMap.appContext;
        final Vision t = this;

        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, openCVLoaderCallback);
            if (!success) {
                Log.e("OpenCV", "Asynchronous initialization failed!");
                error("Could not initialize OpenCV!\r\n" +
                        "Did you install the OpenCV Manager from the Play Store?");
            } else {
                Log.d("OpenCV", "Asynchronous initialization succeeded!");
            }
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            if (openCVLoaderCallback != null)
                openCVLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
            else {
                Log.e("OpenCV", "Failed to load OpenCV from package!");
                return;
            }
        }


    }



    public void loop() {
        VideoCapture capture = new VideoCapture(1);
        Mat frame = new Mat();
        Mat greyframe = new Mat();
        int id = hardwareMap.appContext.getResources().getIdentifier("MyVision", "id", hardwareMap.appContext.getPackageName());
        View view = ((Activity) hardwareMap.appContext).findViewById(id);
        ImageView view1 = (ImageView) view;
        capture.read(frame);
        if(frame.empty() == false){
            Imgproc.cvtColor(frame, greyframe, Imgproc.COLOR_RGB2GRAY);
            Bitmap bm = Bitmap.createBitmap(greyframe.cols(), greyframe.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(greyframe , bm);
            view1.setImageBitmap(bm);
        }

        JavaCameraView camera;
















      /*  List<MatOfPoint> countours = new ArrayList<MatOfPoint>(){};
        Mat Hierarchy = new Mat();
        findContours(greyframe,countours,Hierarchy,RETR_CCOMP, CHAIN_APPROX_SIMPLE);
        //drawContours();
       ColorHSV min = new ColorHSV(255,50,80);
        ColorHSV max = new ColorHSV(205,100,100);
        ColorBlobDetector detector = new ColorBlobDetector(min, max);
        System.out.print(detector.getContours(
        telemetry.addData("getCountours",detector.getContours());
     //   Imgproc.cvtColor(frame, greyframe, Imgproc.COLOR_RGB2GRAY);



        ScheduledExecutorService timer = Executors.newSingleThreadScheduledExecutor();
        timer.scheduleAtFixedRate(run, 0, 33, TimeUnit.MILLISECONDS);*/


    }


}
