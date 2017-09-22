
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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DriveTrainTesting")


public class DriveTrainTesting extends LinearOpMode {
    static DcMotor motor1;
    DriveTrain driveTrain;
    static ColorSensor floorColor;

    static DcMotor rF, rB, lF, lB;
    static GyroSensor gyro;
    private BNO055IMU adaImu;
    private IMU imu;

    static Servo servoMain;
    static Servo servoSec;
    //static GyroSensor gyro;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {

            //    TEST Double REV Hub (look at page 14 of the REV manual)
            //    Plug in 1 motor and 1 servo for each hub

            //    TAKE PIC OF THE TESTING!!!!!!!!

            lB.setPower(0.1);
            Functions.waitFor(3000);
            lB.setPower(0);
            Functions.waitFor(1000);
            rB.setPower(0.1);
            Functions.waitFor(3000);
            rB.setPower(0);
            Functions.waitFor(2000);
            lB.setPower(0.3);
            rB.setPower(0.3);
            Functions.waitFor(3000);
            lB.setPower(0);
            lB.setPower(0);
            Functions.waitFor(3000);
            servoMain.setPosition(0);
            Functions.waitFor(2000);
            servoMain.setPosition(0.9);
            Functions.waitFor(2000);
            servoSec.setPosition(0);
            Functions.waitFor(2000);
            servoSec.setPosition(0.9);
            Functions.waitFor(100000000);
            //    TEST 1

            //test encoder functions once drive train is set up
            driveTrain.moveBkwd(0.2,10,10);
            Functions.waitFor(10000);
            driveTrain.moveFwd(0.2,10,10);
            Functions.waitFor(13000);
            driveTrain.moveRight(0.2,7, 15);
            Functions.waitFor(10000);
            driveTrain.moveLeft(0.2, 7, 15);
            Functions.waitFor(13000);
            driveTrain.moveBkwRight(0.2, 0.2, 8, 10);
            Functions.waitFor(10000);
            driveTrain.moveFwdRight(0.2, 0.2, 8, 10);
            Functions.waitFor(1000000000);

            //    TEST 2   FINISHED

            //test imu.getAngle function by turning robot a few times
            //Turn right by around 90 degrees
            //turn left by around 90 degrees
            //turn left by around 90 degrees again
            //turn left by around 180 degrees
            /*
            int x = 1;
            while(x > 0) {
                telemetry.addData("Heading", imu.getAngle());
                telemetry.update();
                Functions.waitFor(100);
            }

            Functions.waitFor(100000000);

            */


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

        //gyro = hardwareMap.gyroSensor.get("gyro");
        floorColor = hardwareMap.colorSensor.get("floorColor");

        Servo servoMain = hardwareMap.servo.get("servoMain");
        Servo servoSec = hardwareMap.servo.get("servoSec");

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

