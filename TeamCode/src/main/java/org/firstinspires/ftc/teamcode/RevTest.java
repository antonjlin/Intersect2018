
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RevTest")


public class RevTest extends LinearOpMode {
    static DcMotor motor1;
    DriveTrain driveTrain;
    static ColorSensor floorColor;

    static DcMotor rF, rB, lF, lB;
    static GyroSensor gyro;


    static Servo servo1;
    //static GyroSensor gyro;
    @Override
    public void runOpMode() throws InterruptedException {
        Orientation angles;
        initHardware();
        waitForStart();
        while (opModeIsActive()) {

            driveTrain.moveBkwd(0.2,5,10);
            Functions.waitFor(10000);
            driveTrain.moveFwd(0.2,5,10);
            Functions.waitFor(10000000);

            driveTrain.rotateGyroRamp(45, 0.2, 15, gyro, telemetry);
            Functions.waitFor(10000);


            //driveTrain.moveFwd(0.2,5,10);
            //driveTrain.moveFwd();
            //driveTrain.moveFwd(0.2,10,10);
            //driveTrain.moveBkwRight(0.2, 0.2, 5, 10);







        }
    }

    public void initHardware() {
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

