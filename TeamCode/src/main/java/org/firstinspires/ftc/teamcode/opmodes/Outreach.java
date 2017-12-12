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
    static DcMotor rIntake, lIntake, lSlide, rSlide;
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
            double leftPower = scaleDrive(-gamepad1.left_stick_y);
            double rightPower = scaleDrive(-gamepad1.right_stick_y);
            rightSide.setPower(rightPower);
            leftSide.setPower(leftPower);
        }
    }

    private double scaleDrive(double x) {
        if (x < .1 && x > -.1) {
            // Game pad has a little drift. At resting, it may return +/-0,05, set a dead zone here.
            return 0.0;
        }
        // limit to +/- 1.0 motor power
        if (x > 1) {
            return (1);
        }
        if (x < -1) {
            return (-1);
        }
        return x;
    }

    public void initHardware() {
        driveTrain = new DriveTrain(this);
        rIntake = hardwareMap.dcMotor.get("rIntake");
        lIntake = hardwareMap.dcMotor.get("lIntake");
        lSlide = hardwareMap.dcMotor.get("lSlide");
        rSlide = hardwareMap.dcMotor.get("rSlide");
        rightSide = hardwareMap.dcMotor.get("rightSide");
        leftSide = hardwareMap.dcMotor.get("leftSide");

        leftSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSide.setDirection(DcMotorSimple.Direction.REVERSE);
        rIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        lIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        lSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        adaImu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu = new IMU(adaImu);


    }

}

