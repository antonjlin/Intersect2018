
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotutil.VuMark;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VuMarkOpmodeTest")
public class VuMarkOpmodeTest extends LinearOpMode {

    VuMark cryptograph;

    public void runOpMode()throws InterruptedException {

        cryptograph.initVuMark(VuMarkOpmodeTest.this);
        waitForStart();
        if (opModeIsActive()) {
            cryptograph.detectColumn(10);
        }
    }
}

