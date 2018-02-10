
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;


@Autonomous(name = "Drivetrain Test")


public class DriveTrainTesting extends LinearOpMode {
    static DcMotor motor1;
    DriveTrain driveTrain;
    static ColorSensor jewelColor;

    private DcMotor rF, rB, lF, lB;
    private BNO055IMU adaImu;
    IMU imu;
    static Servo servoMain;
    static Servo servoSec;
    //static GyroSensor gyro;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        if (opModeIsActive()) {
            //driveTrain.selfBalance(telemetry);
            //Functions.waitFor(10000);

            //    TEST 1

            //test encoder functions once drive train is set up

            driveTrain.encoderDriveIMU(0.3,30, DriveTrain.Direction.FORWARD,10);
            Functions.waitFor(10000);
            driveTrain.encoderDriveIMU(0.3,30, DriveTrain.Direction.BACKWARD,10);
            driveTrain.encoderDriveIMU(0.3,10, DriveTrain.Direction.LEFT,10);
            Functions.waitFor(10000);
            driveTrain.encoderDriveIMU(0.3,10, DriveTrain.Direction.RIGHT,10);
            /*Functions.waitFor(13000);
            driveTrain.moveRight(0.2,7, 15);
            Functions.waitFor(10000);
            driveTrain.moveLeft(0.2, 7, 15);
            Functions.waitFor(13000);
            driveTrain.moveBkwRight(0.2, 0.2, 8, 10);
            Functions.waitFor(10000);
            driveTrain.moveFwdRight(0.2, 0.2, 8, 10);
            Functions.waitFor(1000000000);
*/
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

            //    TEST 3
*/                /*telemetry.addData("AnglePos", imu.getAnglePositive());
                telemetry.addData("AngleNeg", imu.getAngleNegative());

                telemetry.addData("orentation", imu.getOrientation()[2]);
                telemetry.update();
                }*/
            //test imu turning
            driveTrain.rotateIMURamp(90, 0.2, 10, telemetry);
            //driveTrain.rotateIMURamp(90, 0.2, 10, imu, telemetry);
            Functions.waitFor(5000);
            driveTrain.rotateIMURamp(-180, 0.5, 10, telemetry);
            //driveTrain.rotateIMURamp(-90, 0.2, 10, imu, telemetry);
            Functions.waitFor(5000);
            driveTrain.rotateIMURamp(-90, 1, 10, telemetry);
            //driveTrain.rotateIMURamp(-90, 0.2, 10, imu, telemetry);
            driveTrain.rotateIMURamp(180,0.4,10,telemetry);


            Functions.waitFor(100000000);

            //     TEST 4

            //test self

        }
    }

    public void initHardware() {
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(adaImu);
        servoMain = hardwareMap.servo.get("servoMain");
        servoSec = hardwareMap.servo.get("servoSec");
        driveTrain = new DriveTrain(this);
        this.lB = hardwareMap.dcMotor.get("lB");
        this.rF = hardwareMap.dcMotor.get("rF");
        this.lF = hardwareMap.dcMotor.get("lF");
        this.rB = hardwareMap.dcMotor.get("rB");


    }

}

