package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VuMark;
@Autonomous(name = "Auto Testing")
public class AutoFull extends LinearOpMode {
    static double jewelArmInitPosition = 0, jewelArmDownPos = 0.8, jewelArmUpPos = 0.3 , cryptoDownPos = 0, cryptoUpPos = .5;
    static DcMotor rF, rB, lF, lB, flywheel1, flywheel2, sweeperLow;
    static GyroSensor gyro;
    static Servo jewelArm , cryptoArm , jewelFinger;
    boolean red = false;
    DriveTrain driveTrain;
    ColorSensor jewelColor;
    MRColorSensor colorSensor;
    static double fingerMiddlePos = 0.5;
    static double leftFingerPos = 0.1;
    static double rightFingerPos = 0.0;
    StartingPositions startingPos = StartingPositions.CORNER;
    int state;
    private BNO055IMU adaImu;
    private IMU imu;
    public int crypHeading = 0;
    VuMark vm;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
//        options();
        waitForStart();

        if (opModeIsActive()) {
            driveTrain.alignCrypto(0.5, DriveTrain.Direction.RIGHT);
        }
    }

    public enum StartingPositions {
        CORNER, SANDWITCH;
    }

    public void dumpBlock(double slidePower, double rollerPower, int slideTime, int rollerTime){
        driveTrain.moveSlidesTime(slidePower,slideTime);
        driveTrain.moveRollersTime(rollerPower,slideTime);
        driveTrain.moveSlidesTime(-slidePower,slideTime/2);
    }


    public void fullJewel(){
        jewelArm.setPosition(jewelArmDownPos);
        flickJewel();
        jewelArm.setPosition(jewelArmUpPos);
    }

    public void moveToCrypto(){
        if (colorSensor.team == colorSensor.team.BLUE) {
            driveTrain.encoderDriveIMU(0.3,30, DriveTrain.Direction.BACKWARD,10);
        }
    }

    

    public void flickJewel(){
        if(colorSensor.wrongColor()){
            jewelFinger.setPosition(leftFingerPos);
        }else{
            jewelFinger.setPosition(rightFingerPos);
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

        cryptoArm = hardwareMap.servo.get("cryptoArm");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        jewelFinger = hardwareMap.servo.get("jewelFinger");

        jewelFinger.setPosition(fingerMiddlePos);
        cryptoArm.setPosition(cryptoUpPos);
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
                startingPos = StartingPositions.SANDWITCH;

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
