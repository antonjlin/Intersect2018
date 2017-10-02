
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoPID")
public class AutoPID extends LinearOpMode {
    static DcMotor rFmotor, rBmotor, lFmotor, lBmotor;
    private BNO055IMU adaImu;
    private IMU imu;

    // PID Functions (NEEDS TO BE TUNED)
    private final double PIDKp = 0.0005;
    private final double PIDTi = 1.0;
    private final double PIDTd = 0;
    private final double PIDMax = 1.0;
    private final double PIDMin = 0.2;

    static final double TICKS_PER_INCH_FORWARD = 56;

    // Test Code
    private double averagePowerCumulative = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        waitForStart();

        turnByImuPID(90);

        telemetry.addData(" ", averagePowerCumulative);
        telemetry.update();

        Functions.waitFor(50000);


    }

    public void initHardware() {
        rFmotor = hardwareMap.dcMotor.get("rF");
        rBmotor = hardwareMap.dcMotor.get("rB");
        lFmotor = hardwareMap.dcMotor.get("lF");
        lBmotor = hardwareMap.dcMotor.get("lB");

        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(adaImu);

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
    }

    private void turnByImuPID (int targetHeading) {

        int i = 0;

        PID ImuTurn = new PID(PIDKp, PIDTi, PIDTd, PIDMin, PIDMax);

        double prevTime = System.nanoTime() / 1000000;

        double leftPower;
        double rightPower;

        while (!((int) imu.getAngle() < targetHeading + 5 && (int) imu.getAngle() > targetHeading - 5)) {

            i++;

            int currentHeading = (int) imu.getAngle();
            double deltaTime = (System.nanoTime() / 1000000) - prevTime;
            prevTime = System.nanoTime() / 1000000;

            leftPower = ImuTurn.update(targetHeading, imu.getAngle(), deltaTime);
            rightPower = -ImuTurn.update(targetHeading, imu.getAngle(), deltaTime);

            lFmotor.setPower(leftPower);
            lBmotor.setPower(leftPower);
            rFmotor.setPower(rightPower);
            rBmotor.setPower(rightPower);

            averagePowerCumulative += leftPower;

            telemetry.addData("currentHeading: ", currentHeading);
            telemetry.update();

            Functions.waitFor(25);

        }

        lFmotor.setPower(0);
        lBmotor.setPower(0);
        rFmotor.setPower(0);
        rBmotor.setPower(0);

        averagePowerCumulative /= i;

    }

    private void driveForwardPID(double distance, double power, double timeOutMs) {

        PID headingControl = new PID(PIDKp, PIDTi, PIDTd, PIDMin, PIDMax);

        int targetHeading = (int) imu.getAngle()
                ;
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

            int currentHeading = Math.round(((int) imu.getAngle()));

            double deltaTime = (System.nanoTime() * 1000000) - prevTime;

            double percentTravelled = distanceTraveled/distance;

            boolean turning = false;

            if (targetHeading < currentHeading - 3) {
                // PID Control if robot drifted clockwise (right)

                turning = true;

                leftPower = -headingControl.update(targetHeading, currentHeading, deltaTime);
                rightPower = headingControl.update(targetHeading, currentHeading, deltaTime);

            } else if (targetHeading > currentHeading + 3) {
                // PID Control if robot drifted counterclockwise (left)

                turning = true;

                leftPower = headingControl.update(targetHeading, currentHeading, deltaTime);
                rightPower = -headingControl.update(targetHeading, currentHeading, deltaTime);

            }

            if (turning) {
                lFmotor.setPower(leftPower);
                lBmotor.setPower(leftPower);
                rFmotor.setPower(rightPower);
                rBmotor.setPower(rightPower);
            }
            else {
                leftPower = percentTravelled * leftPower;
                rightPower = percentTravelled * rightPower;

                lFmotor.setPower(leftPower);
                lBmotor.setPower(leftPower);
                rFmotor.setPower(rightPower);
                rBmotor.setPower(rightPower);
            }

            // Calculate distance travelled
            distanceTraveledLeft = (lFmotor.getCurrentPosition() - prevLeftEncoderPosition) / TICKS_PER_INCH_FORWARD;
            distanceTraveledRight = (rFmotor.getCurrentPosition() - prevRightEncoderPosition) / TICKS_PER_INCH_FORWARD;
            distanceTraveled = distanceTraveled + Math.min(distanceTraveledLeft, distanceTraveledRight);

            // Reset
            prevLeftEncoderPosition = lFmotor.getCurrentPosition();
            prevRightEncoderPosition = rFmotor.getCurrentPosition();
            prevTime = System.nanoTime() * 1000000;

            telemetry.addData("currentHeading: ", currentHeading);
            telemetry.update();

            Functions.waitFor(loopTime);

        }

    }

}
