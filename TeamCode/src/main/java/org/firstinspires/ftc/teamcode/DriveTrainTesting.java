
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RevTest")


public class DriveTrainTesting extends LinearOpMode {
    static DcMotor motor1;
    DriveTrain driveTrain;
    static ColorSensor floorColor;

    static DcMotor rF, rB, lF, lB;
    static GyroSensor gyro;
    private BNO055IMU adaImu;
    private IMU imu;

    static Servo servo1;
    //static GyroSensor gyro;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {

            //    TEST 1
            //test encoder functions once drive train is set up
            driveTrain.moveBkwd(0.2,5,10);
            Functions.waitFor(10000);
            driveTrain.moveFwd(0.2,5,10);
            Functions.waitFor(10000000);

            //    TEST 2
            //test imu.getAngle function by turning robot a few times
            //Turn right by around 90 degrees
            telemetry.addData("Heading", imu.getAngle());
            telemetry.update();
            Functions.waitFor(5000);
            //turn left by around 90 degrees
            telemetry.addData("Heading", imu.getAngle());
            telemetry.update();
            Functions.waitFor(5000);
            //turn left by around 90 degrees again
            telemetry.addData("Heading", imu.getAngle());
            telemetry.update();
            Functions.waitFor(5000);
            //turn left by around 180 degrees
            telemetry.addData("Heading", imu.getAngle());
            telemetry.update();


            Functions.waitFor(100000000);
            //    TEST 3
            //test imu turning
            driveTrain.rotateIMURamp(90, 0.2, 10, telemetry);
            Functions.waitFor(5000);
            driveTrain.rotateIMURamp(-90, 0.2, 10, telemetry);
            Functions.waitFor(5000);
            driveTrain.rotateIMURamp(-90, 0.2, 10, telemetry);

            Functions.waitFor(100000000);

        }
    }

    public void initHardware() {

        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(adaImu);

        gyro = hardwareMap.gyroSensor.get("gyro");
        floorColor = hardwareMap.colorSensor.get("floorColor");


        rF = hardwareMap.dcMotor.get("rF");
        rB = hardwareMap.dcMotor.get("rB");
        lF = hardwareMap.dcMotor.get("lF");
        lB = hardwareMap.dcMotor.get("lB");
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.FORWARD);
        lB.setDirection(DcMotor.Direction.REVERSE);
        lF.setDirection(DcMotor.Direction.REVERSE);
        driveTrain = new DriveTrain(lB, rB, lF, rF, this, gyro, floorColor);

    }

}

