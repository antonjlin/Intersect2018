
package org.firstinspires.ftc.teamcode;
//import com.qualcomm.hardware.adafruit.BNO055IMU;
//import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous ;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IMUtest")
public class IMUtest extends LinearOpMode {
        BNO055IMU imu;
        Orientation angles;
    static DcMotor rF, rB, lF, lB;
        //Acceleration gravity;
        @Override
        public void runOpMode() {
            lB = hardwareMap.dcMotor.get("dltM");
            lF = hardwareMap.dcMotor.get("dlbM");
            rF = hardwareMap.dcMotor.get("drtM");
            lF = hardwareMap.dcMotor.get("drbM");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//Send telemetry message to signify robot waiting
            telemetry.addData("Status", "Resetting Encoders");
            telemetry.update();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            composeIMUTelemetry(); //set up telemetry for gyro
//All code above is begun upon init; all code below is begun upon start
            waitForStart();
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            while (Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) < 90) { //imu.getAngularOrientation().firstAngle
                lB.setPower(0.3);
                lF.setPower(0.3);
                rB.setPower(-0.3);
                rF.setPower(-0.3);
            }
            lB.setPower(0);
            lF.setPower(0);
            rB.setPower(0);
            rF.setPower(0);
            while(opModeIsActive()) {
                telemetry.update();
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

    }

        public void composeIMUTelemetry() {

// At the beginning of each telemetry update, grab a bunch of data
// from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() { @Override public void run()
            {
// Acquiring the angles is relatively expensive; we don't want
// to do that in each of the three items that need that info, as that's
// three times the necessary expense.
                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    });
        }


//----------------------------------------------------------------------------------------------
// Formatting
//----------------------------------------------------------------------------------------------

        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit , angle));
        }

        String formatDegrees(double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }
