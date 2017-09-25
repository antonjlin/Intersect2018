
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VuMarkOpmodeTest")
public class VuMarkOpmodeTest extends LinearOpMode {

    VuMark cryptograph;

    public void runOpMode() throws InterruptedException {

        cryptograph.initVuMark(VuMarkOpmodeTest.this);
        waitForStart();
        if (opModeIsActive()) {
            cryptograph.detectColumn(10);
        }
    }
}

