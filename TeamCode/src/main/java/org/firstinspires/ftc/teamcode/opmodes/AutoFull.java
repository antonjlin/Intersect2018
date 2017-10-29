
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.lasarobotics.vision.util.color.Color;

@Autonomous(name = "AutoFull")
public class AutoFull extends LinearOpMode {
    private double jewelArmInitPosition = 0, jewelArmDownPos = 1, getJewelArmUpPos = 0.4;
    static DcMotor rF, rB, lF, lB, flywheel1, flywheel2, sweeperLow;
    static GyroSensor gyro;
    static Servo jewelArm;
    boolean red = false;
    DriveTrain driveTrain;
    ColorSensor jewelColor;
    MRColorSensor colorSensor;
    char alliance;
    int state;
    private BNO055IMU adaImu;
    private IMU imu;
    public int crypHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        options();
        waitForStart();
        state = 0;// Todo:
        if (opModeIsActive()) {
            colorSensor.enableLED(false);
            jewelArm.setPosition(jewelArmDownPos);
            Functions.waitFor(5000);
            telemetry.addData("Red", colorSensor.getRed());
            telemetry.addData("Blue", colorSensor.getBlue());
            hitJewel(0.2,2);
            Functions.waitFor(5000);
            driveTrain.encoderDriveIMU(0.4,30, DriveTrain.Direction.FORWARD,10);
            driveTrain.rotateIMURamp(90,0.3,5,telemetry);
        }
    }

    public void hitJewel(double speed, int dist){
        if(colorSensor.correctColor()){
            driveTrain.encoderDriveIMU(speed,dist, DriveTrain.Direction.BACKWARD,3);
            driveTrain.encoderDriveIMU(speed,dist, DriveTrain.Direction.FORWARD,3);
        }else{
            driveTrain.encoderDriveIMU(speed,dist, DriveTrain.Direction.FORWARD,3);
            driveTrain.encoderDriveIMU(speed,dist, DriveTrain.Direction.BACKWARD,3);
        }
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
        colorSensor = new MRColorSensor(jewelColor, this);
        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        //crypHeading = (int) imu.getAngle() - 90;
        driveTrain = new DriveTrain(this);
        driveTrain.detectAmbientLight(jewelColor);

        //driveTrain.calibrateGyro(telemetry);

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
