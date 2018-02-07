package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by ani on 12/26/17.
 */
@TeleOp
public class IntakeTest extends LinearOpMode{
    DcMotor lIntake;
    DcMotor rIntake;

    public void runOpMode(){
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            lIntake.setPower(y);
            rIntake.setPower(y);
            telemetry.addData("yval", y);
            telemetry.update();
        }

    }

    void initialize(){
        lIntake = hardwareMap.dcMotor.get("lIntake");
        rIntake = hardwareMap.dcMotor.get("rIntake");
        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rIntake.setDirection(DcMotorSimple.Direction.FORWARD);

    }



}
