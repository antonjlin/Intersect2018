
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Full Mission")
public class AutoFull extends LinearOpMode {
    static DcMotor rF, rB, lF, lB, flywheel1, flywheel2, sweeperLow;
    static GyroSensor gyro;
    static ColorSensor beaconColor,floorColor;
    static int conversionFactor = 50;
    static Servo sideWall;
    static CRServo buttonPusher;
    boolean red = false;
    boolean center = true;
    boolean timedOut = false;
    boolean getBeacons = true;
    double average;
    DriveTrain driveTrain;
    char alliance;
    String parkCenter;
    int state;
    int beaconAction = 0;
    int ambientBlue = 0;
    int ambientRed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 0.2;
        int gyroRampMax = 20;
        int gyroRampMin = 3;
        int shootingPositionDistance = 15;
        int wallPositionDriveDistance = 50;
        initHardware();
        waitForStart();
        state = 0;// Todo:
        if (opModeIsActive()) {

        }
    }



    public void initHardware() {
        buttonPusher = hardwareMap.crservo.get("buttonPusher");
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


        flywheel1 = hardwareMap.dcMotor.get("flywheel1");
        flywheel2 = hardwareMap.dcMotor.get("flywheel2");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        //beaconColor = hardwareMap.colorSensor.get("beaconColor");
        floorColor = hardwareMap.colorSensor.get("floorColor");
        beaconColor = hardwareMap.colorSensor.get("beaconColor");
        sweeperLow = hardwareMap.dcMotor.get("sweeperLow");
        sideWall = hardwareMap.servo.get("sideWall");
        sideWall.setPosition(Functions.sideWallUpPos);
        beaconColor.setI2cAddress(I2cAddr.create8bit(0x3c));
        gyro = hardwareMap.gyroSensor.get("gyro");
        driveTrain = new DriveTrain(lB, rB, lF, rF, this, gyro, floorColor);
        driveTrain.detectAmbientLight(beaconColor);
        driveTrain.calibrateGyro(telemetry);
    }

    public void options(){
        telemetry.addData("Team", "Blue");
        telemetry.update();
        boolean confirmed = false;
        red = false;
        while(!confirmed){
            if (gamepad1.a){
                red = true;
                telemetry.addData("Team", red ? "Red": "Blue");
            }
            if (gamepad1.b){
                red = false;
                telemetry.addData("Team", red ? "Red": "Blue");
            }
            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Team", red ? "Red" : "Blue");
                telemetry.addData("Confirmed!", "");
                telemetry.update();
                confirmed = true;
            }

        }
    }

    private void haltUntilPressStart() {
        while (!gamepad1.start  && !isStopRequested()) {
            Functions.waitFor(300);
        }
    }
}
