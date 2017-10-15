
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotutil.VuMark;


@Autonomous(name = "vumark opmode test")
public class VuMarkOpmodeTest extends LinearOpMode {


    public void runOpMode()throws InterruptedException {
        VuMark cryptograph = new VuMark(this);
        waitForStart();
        if (opModeIsActive()) {
            cryptograph.detectColumn(30);
        }
    }
}

