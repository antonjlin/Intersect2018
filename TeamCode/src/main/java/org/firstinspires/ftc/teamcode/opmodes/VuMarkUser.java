package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robotutil.VuMarkGetter;


public class VuMarkUser extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuMarkGetter vuMarkGetter = new VuMarkGetter();
        VuforiaTrackables trackables = vuMarkGetter.getVuforia();
        waitForStart();

        VuMarkGetter.DistanceOffsets distanceOffsets;
        while (opModeIsActive()) {
            distanceOffsets = vuMarkGetter.getOffset(trackables);
            telemetry.addData("Target Type", distanceOffsets.vuMarkType.toString());
            telemetry.addData("Distance", distanceOffsets.distance);
            telemetry.addData("Horizontal", distanceOffsets.horizontal);
            telemetry.addData("Vertical", distanceOffsets.vertical);
            telemetry.update();
        }

    }
}