
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoPID")
public class AutoPID extends LinearOpMode {
    static DcMotor rFmotor, rBmotor, lFmotor, lBmotor;
    private BNO055IMU adaImu;

    Orientation angle;

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

        int x = 0;

        while (x == 0) {
            telemetry.addData("", adaImu.getAngularOrientation().firstAngle);
            telemetry.addData("", adaImu.getAngularOrientation().secondAngle);
            telemetry.addData("", adaImu.getAngularOrientation().thirdAngle);

            Functions.waitFor(500);
        }


    }

    public void initHardware() {
        rFmotor = hardwareMap.dcMotor.get("rF");
        rBmotor = hardwareMap.dcMotor.get("rB");
        lFmotor = hardwareMap.dcMotor.get("lF");
        lBmotor = hardwareMap.dcMotor.get("lB");

        adaImu = hardwareMap.get(BNO055IMU.class, "imu");

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

        PID ImuTurn = new PID(PIDKp, PIDTi, PIDTd, -PIDIntMax, PIDIntMax);

        double prevTime = 0;

        int loopTime = 50; // Loops every 50 ms
        double startTime = System.nanoTime() * 1000000;

        double leftPower;
        double rightPower;

        while (adaImu.getAngularOrientation().firstAngle != targetHeading) {

            int currentHeading = Math.round(angle.firstAngle);
            double deltaTime = (System.nanoTime() * 1000000) - prevTime;
            leftPower = ImuTurn.update(targetHeading, adaImu.getAngularOrientation().firstAngle, deltaTime);
            rightPower = -ImuTurn.update(targetHeading, adaImu.getAngularOrientation().firstAngle, deltaTime);

            lFmotor.setPower(leftPower);
            lBmotor.setPower(leftPower);
            rFmotor.setPower(rightPower);
            rBmotor.setPower(rightPower);

        }

    }

    private void driveForwardPID(double distance, double power, double timeOutMs) {

        PID headingControl = new PID(PIDKp, PIDTi, PIDTd, -PIDIntMax, PIDIntMax);

        int targetHeading = Math.round(angle.firstAngle);
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

            int currentHeading = Math.round(adaImu.getAngularOrientation().firstAngle);

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
