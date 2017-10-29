
package org.firstinspires.ftc.teamcode.opmodes;

import android.app.Activity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.VisionProcessing;

import static org.firstinspires.ftc.teamcode.robotutil.DriveTrain.Direction.*;

@Autonomous(name = "AutoFull")
public class AutoFull extends LinearOpMode {
    static  DcMotor rF, rB, lF, lB, flywheel1, flywheel2, sweeperLow;
    static Servo rServo;
    static GyroSensor gyro;
    static ColorSensor jewelColor;
    boolean red = false;
    boolean redBallIsLeft;
    char alliance;
    int state;
    private BNO055IMU adaImu;
    private IMU imu;
    public int crypHeading = 0;

    DriveTrain driveTrain = new DriveTrain(this);


    @Override
    public void runOpMode() throws InterruptedException {
        VisionProcessing processing = new VisionProcessing((Activity) hardwareMap.appContext);
        initHardware();
        waitForStart();
        state = 0;// Todo:
        if(red) {
            if (redBallIsLeft) driveTrain.encoderDrive(.3, 4, BACKWARD, 5000);
            else driveTrain.encoderDrive(.3, 4, FORWARD, 5000);

            driveTrain.encoderDrive(.3, 12, FORWARD,5000); //until wall detected

            driveTrain.encoderDrive(.3, 4,LEFT,5000); //until
            //detect cryptobox/align


        }
    }



    public void initHardware() {
        rF = hardwareMap.dcMotor.get("rF");
        rB = hardwareMap.dcMotor.get("rB");
        lF = hardwareMap.dcMotor.get("lF");
        lB = hardwareMap.dcMotor.get("lB");
        rServo = hardwareMap.servo.get("rServo");
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
            adaImu = hardwareMap.get(BNO055IMU.class, "imu");
            imu = new IMU(adaImu);
            jewelColor = hardwareMap.colorSensor.get("jewelColor");
            sweeperLow = hardwareMap.dcMotor.get("sweeperLow");
            crypHeading = (int) imu.getAngle() - 90;

            gyro = hardwareMap.gyroSensor.get("gyro");
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
