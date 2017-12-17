package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.IMU;


/**
 * Created by ani on 12/9/17.
 */
@TeleOp(name = "Outreach")

public class Outreach extends LinearOpMode {
    static DcMotor rightSide, leftSide;
    DriveTrain driveTrain;
    static GyroSensor gyro;
    static ColorSensor floorColor;
    static DcMotor rIntake, lIntake, lSlide, rSlide,lWheel,rWheel;
    private BNO055IMU adaImu;
    private IMU imu;
    int lSlidePos;
    int rSlidePos;
    int slideTicksPerInch;
    int pos0 = 0;
    int pos1 = 6;
    int pos2 = 12;
    int pos3 = 18;
    int pos4 = 24;


    // RampFlywheel rampFlywheel = new RampFlywheel();
    // RampDownFlywheel rampDownFlywheel = new RampDownFlywheel();
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            double leftPower = scaleDrive(gamepad1.left_stick_y);
            double rightPower = scaleDrive(gamepad1.right_stick_y);
            rightSide.setPower(rightPower);
            leftSide.setPower(leftPower);
        }
    }

    private double scaleDrive(double x) {

        // limit to +/- 1.0 motor power
        if (x > .1) {
            return (1);
        }
        if (x < -.1) {
            return (-1);
        }
        return x;
    }

    public void initHardware() {

      lWheel = hardwareMap.dcMotor.get("leftSide");
      rWheel = hardwareMap.dcMotor.get("rightSide");

    }

}

