package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VuMark;
@Autonomous(name = "GarbAuto")
public class AutoFull extends LinearOpMode {
    static double jewelArmInitPosition = .3, jewelArmDownPos = 0.8, jewelArmUpPos = 0.35 , cryptoDownPos = 0, cryptoUpPos = .5;
    static DcMotor rF, rB, lF, lB;
    static GyroSensor gyro;
    static Servo jewelArm;
    static Servo flipServo;
    static Servo jewelFinger;
    boolean red = false;
    DriveTrain driveTrain;
    ColorSensor jewelColor;
    MRColorSensor colorSensor;
    StartingPositions startingPos = StartingPositions.CORNER;
    int state;
    private BNO055IMU adaImu;
    private IMU imu;
    public int crypHeading = 0;
    VuMark vm;
    RelicRecoveryVuMark vumark;
    Boolean garb = false;
    TouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
       options();
        waitForStart();
        if (opModeIsActive()) {
            jewelArm.setPosition(jewelArmDownPos);
            if(driveTrain.detectRed(jewelColor)){
                jewelFinger.setPosition(1);
                Functions.waitFor(200);
                jewelFinger.setPosition(.5);
            }
            else{
                jewelFinger.setPosition(0);
                Functions.waitFor(200);
                jewelFinger.setPosition(.5);
            }
            jewelArm.setPosition(jewelArmDownPos);
            Functions.waitFor(1000);
            if (red) {

                if (startingPos == StartingPositions.CORNER) {
                    driveTrain.encoderDrive(0.4, 10, DriveTrain.Direction.FORWARD, 3);
                    vumark = vm.detectColumn(5);
                    driveTrain.encoderDrive(.4, 10, DriveTrain.Direction.FORWARD, 4);
                    driveTrain.rotateIMURamp(-90, .4,1, this.telemetry);
                    driveTrain.moveUntilTouchRed(5000);
                    driveTrain.columnBlockRed(vumark);

                } else {

                    driveTrain.encoderDrive(0.5, 10, DriveTrain.Direction.BACKWARD, 10);
                    vumark = vm.detectColumn(5);
                }

            } else {
                if (startingPos == StartingPositions.CORNER) {
                    driveTrain.encoderDrive(0.4, 10, DriveTrain.Direction.FORWARD, 3);
                    vumark = vm.detectColumn(5);
                    driveTrain.encoderDrive(.4, 25, DriveTrain.Direction.BACKWARD, 4);
                    driveTrain.rotateIMURamp(-90, .5, 5, telemetry);
                    driveTrain.moveUntilTouchBlue(5000);
                    driveTrain.columnBlockBlue(vumark);
                    
                } else {
                    flipServo.setPosition(0);
                    driveTrain.encoderDrive(0.5, 60, DriveTrain.Direction.BACKWARD, 10);
                    driveTrain.encoderDrive(0.5, 10, DriveTrain.Direction.FORWARD, 10);
                }

            }
        }
    }

    public void redCorner(){
        driveTrain.encoderDrive(0.4, 26, DriveTrain.Direction.FORWARD, 10);
        driveTrain.rotateIMURamp(-90, .5, 5, telemetry);
        flipServo.setPosition(0);
        driveTrain.encoderDrive(0.5, 20, DriveTrain.Direction.BACKWARD, 10);
        driveTrain.encoderDrive(0.5, 10, DriveTrain.Direction.FORWARD, 10);
    }
    public void blueCorner(){
        driveTrain.encoderDrive(0.4, 26, DriveTrain.Direction.BACKWARD, 10);
        driveTrain.rotateIMURamp(-90, .5, 5, telemetry);
        flipServo.setPosition(0);
        driveTrain.encoderDrive(0.5, 20, DriveTrain.Direction.BACKWARD, 10);
        driveTrain.encoderDrive(0.5, 10, DriveTrain.Direction.FORWARD, 10);
    }
    public void redSandwich(){
        flipServo.setPosition(0);
        driveTrain.encoderDrive(0.5, 60, DriveTrain.Direction.BACKWARD, 10);
        driveTrain.encoderDrive(0.5, 10, DriveTrain.Direction.FORWARD, 10);
    }
    public void blueSandwich(){
        flipServo.setPosition(0);
        driveTrain.encoderDrive(0.5, 60, DriveTrain.Direction.BACKWARD, 10);
        driveTrain.encoderDrive(0.5, 10, DriveTrain.Direction.FORWARD, 10);
    }

    public enum StartingPositions {
        CORNER, SANDWICH;
    }
    public void knockJewel() {
        driveTrain.encoderDriveIMU(.25,3, DriveTrain.Direction.FORWARD, 5);
        driveTrain.encoderDriveIMU(.25,-3, DriveTrain.Direction.FORWARD, 5);
    }

    public void dumpBlock(double slidePower, double rollerPower, int slideTime, int rollerTime){
        driveTrain.moveSlidesTime(slidePower,slideTime);
        driveTrain.moveRollersTime(rollerPower,slideTime);
        driveTrain.moveSlidesTime(-slidePower,slideTime/2);
    }


    public void fullJewel(){
        jewelArm.setPosition(jewelArmDownPos);
        jewelArm.setPosition(jewelArmUpPos);
    }

    public void moveToCrypto(){
        if (colorSensor.team == colorSensor.team.BLUE) {
            driveTrain.encoderDriveIMU(0.3,30, DriveTrain.Direction.BACKWARD,10);
        }
    }

    public void dumpBlock(){

    }


    public void initHardware() {
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

//        cryptoArm = hardwareMap.servo.get("cryptoArm");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        touch = hardwareMap.touchSensor.get("touch");
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");

        flipServo = hardwareMap.servo.get("flipServo");
        flipServo.setDirection(Servo.Direction.REVERSE);


        jewelFinger = hardwareMap.servo.get("jewelFinger");

        jewelFinger.setPosition(.5);
       //cryptoArm.setPosition(cryptoUpPos);
        jewelArm.setPosition(jewelArmInitPosition);

        imu = new IMU(adaImu);

        colorSensor = new MRColorSensor(jewelColor, this);
        driveTrain = new DriveTrain(this);
        driveTrain.detectAmbientLight(jewelColor);
        vm = new VuMark(this);

    }

    public void options(){
        telemetry.addData("Team", "Blue");
        telemetry.update();
        boolean confirmed = false;
        red = false;
        while(!confirmed){
            if (gamepad1.a){
                red = true;
                colorSensor.team = Team.RED;

                telemetry.addData("Team", red ? "Red": "Blue");
            }
            if (gamepad1.b){
                red = false;
                colorSensor.team = Team.BLUE;

                telemetry.addData("Team", red ? "Red": "Blue");
            }
            if (gamepad1.x){
                startingPos = StartingPositions.CORNER;

                telemetry.addData("Starting Position", startingPos == StartingPositions.CORNER ? "corner": "sandwich");
            }
            if (gamepad1.y){
                startingPos = StartingPositions.SANDWICH;

                telemetry.addData("Starting Position", startingPos == StartingPositions.CORNER ? "corner": "sandwich");
            }

            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Team", red ? "Red" : "Blue");
                telemetry.addData("Starting Position", startingPos == StartingPositions.CORNER ? "corner": "sandwich");
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
