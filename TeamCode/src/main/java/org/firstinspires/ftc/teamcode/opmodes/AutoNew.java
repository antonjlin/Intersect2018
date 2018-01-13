package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VuMark;

/**
 * Created by pranav on 1/10/18.
 */

public class AutoNew extends LinearOpMode{
    static double jewelArmInitPosition = 0, jewelArmDownPos = 0.96, jewelArmUpPos = 0.3;
    static DcMotor rF, rB, lF, lB, flywheel1, flywheel2, sweeperLow;
    static GyroSensor gyro;
    static Servo jewelArm;
    boolean red = false;
    DriveTrain driveTrain;
    ColorSensor jewelColor;
    ColorSensor cryptoColor;
    MRColorSensor colorSensor;
    int startingPos = 0; //0 IS CORNER POSITION, 1 IS SANDWITCH POSITION
    int state;
    private BNO055IMU adaImu;
    private IMU imu;
    public int crypHeading = 0;
    VuMark vm;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        options();
        waitForStart();

        if (opModeIsActive()) {


            if (startingPos == 0){
                if(red){

                    driveTrain.encoderDrive(0.4, 36, DriveTrain.Direction.FORWARD, 10);
                } else{
                    driveTrain.encoderDrive(0.4, 36, DriveTrain.Direction.BACKWARD, 10);
                }
                Functions.waitFor(1000);
                driveTrain.rotateIMURamp(-90,.4, 4, telemetry);
                Functions.waitFor(1000);
                dumpBlock(1,1,800,3000);
                Functions.waitFor(1000);
                driveTrain.encoderDrive(0.5,1000, DriveTrain.Direction.BACKWARD,4);
                driveTrain.encoderDriveIMU(.5, 5, DriveTrain.Direction.FORWARD, 3 );
                driveTrain.encoderDrive(0.5,1000, DriveTrain.Direction.BACKWARD,4);


            }
            else{

                driveTrain.rotateIMURamp(30,.4, 4, telemetry);
                Functions.waitFor(1000);

                driveTrain.encoderDrive(0.4, 15, DriveTrain.Direction.FORWARD, 10);
                Functions.waitFor(1000);

                if(!red) {
                    driveTrain.rotateIMURamp(180,.4, 10, telemetry);
                    Functions.waitFor(1000);

                }
                dumpBlock(1,1,800,3000);
                Functions.waitFor(1000);
                driveTrain.encoderDrive(0.5,1000, DriveTrain.Direction.BACKWARD,2);
                driveTrain.encoderDrive(.5, 5, DriveTrain.Direction.LEFT, 2);
                driveTrain.encoderDriveIMU(.5, 5, DriveTrain.Direction.FORWARD, 3 );
                driveTrain.encoderDrive(0.5,1000, DriveTrain.Direction.BACKWARD,4);

            }

        }
    }

    public void dumpBlock(double slidePower, double rollerPower, int slideTime, int rollerTime){
        driveTrain.moveSlidesTime(slidePower,slideTime);
        driveTrain.moveRollersTime(rollerPower,slideTime);
        driveTrain.moveSlidesTime(-slidePower,slideTime/2);
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
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelArm.setPosition(jewelArmInitPosition);
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(adaImu);
        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        //cryptoColor = hardwareMap.colorSensor.get("cryptoColor");
        //colorSensor = new MRColorSensor(jewelColor, this);
        colorSensor = new MRColorSensor(jewelColor);
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
                startingPos = 0;

                telemetry.addData("Starting Position", startingPos ==0 ? "corner": "sandwich");
            }
            if (gamepad1.y){
                startingPos = 1;

                telemetry.addData("Starting Position", startingPos ==0 ? "corner": "sandwich");
            }

            telemetry.update();

            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                telemetry.addData("Team", red ? "Red" : "Blue");
                telemetry.addData("Starting Position", startingPos ==0 ? "corner": "sandwich");
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
