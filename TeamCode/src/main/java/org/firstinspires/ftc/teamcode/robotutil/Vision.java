package org.firstinspires.ftc.teamcode.robotutil;

import android.app.Activity;
import android.graphics.Bitmap;
import android.view.View;
import android.widget.ImageView;

import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.opencv.android.JavaCameraView;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by pranav on 10/22/17.
 */

public class Vision extends ManualVisionOpMode{
    ImageView view;
    Mat rgba;
    Mat gray;


    public Vision(){


        initialize();
    }

    public Mat frame(Mat rgba,Mat gray) {
        this.rgba = rgba;
        this.gray = gray;
        Bitmap bm = Bitmap.createBitmap(rgba.cols(), rgba.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(rgba, bm);
        view.setImageBitmap(bm);
        return null;
    }
    public void findCircles(){


    }
    public void loop(){


        /*cameraView = (JavaCameraView) ((Activity) hardwareMap.appContext).findViewById(id);
        cameraView.setVisibility(View.VISIBLE);
        cameraView.setCvCameraViewListener(this);
        cameraView.setCameraIndex(98);
        cameraView.enableView();*/
    }

    void initialize(){
        int id = hardwareMap.appContext.getResources().getIdentifier("MyVision", "id", hardwareMap.appContext.getPackageName());
        view = (ImageView) ((Activity) hardwareMap.appContext).findViewById(id);
        view.setVisibility(View.VISIBLE);

    }

}
