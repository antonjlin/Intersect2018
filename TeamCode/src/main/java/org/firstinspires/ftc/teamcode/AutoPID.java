
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoPID")
public class AutoPID extends LinearOpMode {
    static DcMotor rFmotor, rBmotor, lFmotor, lBmotor;
    static GyroSensor gyro;

    // PID Functions (NEEDS TO BE TUNED)
    private final double PIDKp = 0.004;
    private final double PIDTi = 0.4;
    private final double PIDTd = 25;
    private final double PIDIntMax = 100;

    static final double TICKS_PER_INCH_FORWARD = 56;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        waitForStart();

    }

    public void initHardware() {
        rFmotor = hardwareMap.dcMotor.get("rF");
        rBmotor = hardwareMap.dcMotor.get("rB");
        lFmotor = hardwareMap.dcMotor.get("lF");
        lBmotor = hardwareMap.dcMotor.get("lB");

        // Need to add gyro to config file, X is placeholder
        gyro = hardwareMap.gyroSensor.get("X");

        lBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lFmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        calibrateGyro(telemetry);
    }

    /***
     *     Gyro calibration routing
     *     @param telemetry - telemetry printing handle
     *
      */

    private void calibrateGyro(Telemetry telemetry){
        gyro.calibrate();
        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating() && !isStopRequested())  {
            Functions.waitFor(50);
        }
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    private void driveForwardPID(double distance, double power, double timeOutMs) {

        PID headingControl = new PID(PIDKp, PIDTi, PIDTd, -PIDIntMax, PIDIntMax);

        int targetHeading = gyro.getHeading();
        double distanceTraveled = 0; // in inches
        double distanceTraveledLeft;
        double distanceTraveledRight;

        int prevLeftEncoderPosition = 0;
        int prevRightEncoderPosition = 0;
        double prevTime = 0;

        int loopTime = 50; // Loops every 50 ms
        double startTime = System.nanoTime() * 1000000;

        double leftPower = power;
        double rightPower = power;

        while (distance > distanceTraveled && (System.nanoTime() * 1000000) - startTime < timeOutMs) {

            int currentHeading = gyro.getHeading();
            double deltaTime = (System.nanoTime() * 1000000) - prevTime;

            if (targetHeading < gyro.getHeading() - 3) {
                // PID Control if robot drifted clockwise (right)

                leftPower = -headingControl.update(targetHeading, gyro.getHeading(), deltaTime);
                rightPower = headingControl.update(targetHeading, gyro.getHeading(), deltaTime);

            } else if (targetHeading > gyro.getHeading() + 3) {
                // PID Control if robot drifted counterclockwise (left)

                leftPower = headingControl.update(targetHeading, gyro.getHeading(), deltaTime);
                rightPower = -headingControl.update(targetHeading, gyro.getHeading(), deltaTime);

            }

            lFmotor.setPower(leftPower);
            lBmotor.setPower(leftPower);
            rFmotor.setPower(rightPower);
            rBmotor.setPower(rightPower);

            // Calculate distance travelled
            distanceTraveledLeft = (lFmotor.getCurrentPosition() - prevLeftEncoderPosition) / TICKS_PER_INCH_FORWARD;
            distanceTraveledRight = (rFmotor.getCurrentPosition() - prevRightEncoderPosition) / TICKS_PER_INCH_FORWARD;
            distanceTraveled = distanceTraveled + Math.min(distanceTraveledLeft, distanceTraveledRight);

            // Reset
            prevLeftEncoderPosition = lFmotor.getCurrentPosition();
            prevRightEncoderPosition = rFmotor.getCurrentPosition();
            prevTime = System.nanoTime() * 1000000;

            Functions.waitFor(loopTime);

        }

    }

}
