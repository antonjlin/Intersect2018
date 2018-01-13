package org.firstinspires.ftc.teamcode.opmodes;

import android.app.Activity;
import android.view.animation.LinearInterpolator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotutil.BallDetector;
import org.firstinspires.ftc.teamcode.robotutil.VisionAlign;
import org.firstinspires.ftc.teamcode.robotutil.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Created by pranav on 10/22/17.
 */
@TeleOp
public class VisionTesting extends LinearOpMode {

    public void runOpMode() {


        VisionAlign visionAlign = new VisionAlign();
        waitForStart();
        VisionProcessor processor = new VisionProcessor((Activity) this.hardwareMap.appContext, visionAlign );
        double[] d = visionAlign.getVals();
        //Mat RGB = processor.getMatRGB();

        //VisionAlign align = new VisionAlign();
        while (opModeIsActive()) {
            //double[] d = align.lineDetect(RGB);
              /*if (RGB == null) {
                telemetry.addData("status", "  null");
            } else {
                telemetry.addData("status", "  notnull");


            }*/
            telemetry.update();
           if(d == null){
               telemetry.addData("status  ", "null");
           }
           else {
              for (int i = 0; i < d.length; i++) {
                   telemetry.addData("vals", d[i]);
                   telemetry.addLine();
               }
               telemetry.update();
           }

        }

    }
}
