package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class DriveTrain {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final double TICKS_PER_INCH_FORWARD = 62;
    static final double TICKS_PER_INCH_STRAFE = 61.3;
    public DcMotor rF, rB, lF, lB, rIntake, lIntake, rSlide, lSlide;
    public double minMotorPower = 0.085; //minimum power that robot still moves
    public IMU imu;


    // Tunable parameters

    private int conversionFactor = 50;
    public double balanceThreshold = 1.5;
    public double balanceMultiplier = 0.08;
    private int gyroTurnErrorMargin = 3; // turn stop if within the margin of error
    private int gyroTurnRampMax = 60;  // starting point of scaling back speed of motor for turning
    private int gyroTurnRampMin = 3;   // stopping point to turn off motor abs(heading-target)<vlaue
    private double minRotationPower = 0.03; // minimum power to move robot
    private final int driveStraightErrorMargin = 2;
    private final int encoderDriveRampMax = 40;
    private final int encoderDriveRampMin = 1;
    private int ambientBlue = 0;
    private int ambientRed = 0;

    double average;
    LinearOpMode opMode;

    public DriveTrain( LinearOpMode opMode) {
        this.opMode = opMode;
        lB = opMode.hardwareMap.dcMotor.get("lB");
        lB = opMode.hardwareMap.dcMotor.get("lB");
        rF = opMode.hardwareMap.dcMotor.get("rF");
        lF = opMode.hardwareMap.dcMotor.get("lF");
        rB = opMode.hardwareMap.dcMotor.get("rB");

        rSlide = opMode.hardwareMap.dcMotor.get("rSlide");
        lSlide = opMode.hardwareMap.dcMotor.get("lSlide");
        BNO055IMU adaImu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu = new IMU(adaImu);
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
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        lIntake = opMode.hardwareMap.dcMotor.get("lIntake");
        rIntake = opMode.hardwareMap.dcMotor.get("rIntake");
        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rollersSetPower(double power) {
        rIntake.setPower(power);
        lIntake.setPower(power);
    }

    public void moveRollersTime(double power, int timeMS) {
        rollersSetPower(power);
        Functions.waitFor(timeMS);
        rollersSetPower(0);

    }

    public enum Direction {
        FORWARD, RIGHT, LEFT, BACKWARD;
    }
    public void moveAtSpeed(Direction direction, double speed) {
        if(direction == Direction.FORWARD) {
            lF.setPower(Math.abs(speed));
            lB.setPower(Math.abs(speed));
            rF.setPower(Math.abs(speed));
            rB.setPower(Math.abs(speed));
        }    else    if(direction == Direction.BACKWARD) {
            lF.setPower(-Math.abs(speed));
            lB.setPower(-Math.abs(speed));
            rF.setPower(-Math.abs(speed));
            rB.setPower(-Math.abs(speed));
        }    else    if(direction == Direction.RIGHT) {
            lF.setPower(-Math.abs(speed));
            lB.setPower(Math.abs(speed));
            rF.setPower(Math.abs(speed));
            rB.setPower(-Math.abs(speed));
        }    else    {
            lF.setPower(Math.abs(speed));
            lB.setPower(-Math.abs(speed));
            rF.setPower(-Math.abs(speed));
            rB.setPower(Math.abs(speed));
        }
    }

    public void slidesSetPower(double power) {
        rSlide.setPower(power);
        lSlide.setPower(power);
    }

    private int headingCWError(int start, int turn, int current ) {

        int offset;
        int target;

        offset = 90 - start;
        target = start + turn + offset;
        current += offset;
        if ( current < 0 ) {
            current+=360;
        } // avoid wrap around problem
        return target - current;
    }

    private int headingCCWError(int start, int turn, int current ) {

        int offset;
        int target;

        offset = 270 - start;
        target = start - turn + offset;
        current += offset;
        if ( current >=360 ) {
            current-=360;
        } // avoid wrap around problem
        return target - current;

    }

    private double  headingError (double start, double turn, double current) {

        double e;

        // current angle from start
        double offset = current - start;

        e = turn - offset;

        return e;

    }

    public void SetGryoTurnParameters(int max, int min, double speed){
        gyroTurnRampMax = max;
        gyroTurnRampMin = min;
        minRotationPower = speed;
    }

    private double powerAdjust ( double e ) {
        e = Math.abs(e);
        if ( e > gyroTurnRampMax) {
            return 1.0;
        }
        if ( e <= gyroTurnRampMin ) {
            return 0;
        }
        return (e-gyroTurnRampMin)/(gyroTurnRampMax-gyroTurnRampMin) ;
    }

    private double powerAdjustEncoderDrive ( double e ) {
        e = Math.abs(e);
        if ( e > encoderDriveRampMax) {
            return 1.0;
        }
        else if (e < encoderDriveRampMin) {
            return 0;
        }
        return (e-encoderDriveRampMin)/(encoderDriveRampMax-encoderDriveRampMin) ;
    }

    // @param degree - turn in degree up to +/- 180.   +/- = clockwise/counterclockwise
    // @param power - turn motor power
    // @param timeout - time out in seconds
    // @param gyro pointer to Gyro object
    // @param telemetry - pointer to telemetry object

    public double rotateIMURamp(int degrees, double power, int timeoutS, Telemetry telemetry) {
    //public double rotateIMURamp(int degrees, double power, int timeoutS, IMU imu, Telemetry telemetry) {
        //COUNTERCLOCKWISE IS POSITIVE DEGREES
        double heading;
        int e;
        long endtime = System.currentTimeMillis() + (timeoutS * 1000);
        double start = imu.getAngle();

        double target = start + degrees;
        if ( target < 0) {
            target += 360;
        } else if (target >=360 ) {
            target -=360;
        }
//        setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clear();
        telemetry.addData("Start", start);
        telemetry.addData("Target", target);

        if (Math.abs(degrees) <= gyroTurnErrorMargin ){
            telemetry.addData("Too small to turn ", degrees);
            telemetry.update();
            return start;
        }
        telemetry.update();
        do {
            heading = imu.getAngle();
            if ( degrees > 0 ) { // Turn clockwise
                e = headingCWError((int) start, degrees, (int) heading); // Heading error
            } else { // turn counter clockwise
                e = headingCCWError((int) start, -degrees, (int) heading); // Heading error
            }
            if ( e > 0 ) {
                rotateCW(Math.max(minRotationPower, power * powerAdjust(e)));
            } else {  // overshoot
                rotateCCW(Math.max(minRotationPower, power * powerAdjust(e)));
            }
            if (Math.abs(e) <= gyroTurnErrorMargin) {
                this.stopAll();
                Functions.waitFor(100); // wait for 500 msec.
                heading = imu.getAngle(); // read heading again
                if ( degrees > 0 ) { // Turn clockwise
                    e = headingCWError((int) start, degrees, (int) heading); // Heading error
                } else { // turn counter clockwise
                    e = headingCCWError((int) start, -degrees, (int) heading); // Heading error
                }
            }

            telemetry.clear();
            telemetry.addData("Start", start);
            telemetry.addData("Heading", imu.getAngle());
            telemetry.addData("Target", target);
            telemetry.addData("timeout", timeoutS * 1000);
            telemetry.addData("End Time", endtime);
            telemetry.addData("Error", e);
            telemetry.update();
        } while ((Math.abs(e) > gyroTurnErrorMargin) &&  (System.currentTimeMillis() < endtime) && opMode.opModeIsActive());

        this.stopAll();
        return heading;

//      setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void alignWithCrypto(RelicRecoveryVuMark vuMark, ColorSensor colors){
        switch (vuMark){
            case LEFT:
                encoderDriveIMU(.7, 30, Direction.LEFT, 10);
                driveRighttoCrypto(vuMark,colors);
            case RIGHT:
               // driveLefttoCrypto();

        }
    }

    public void cryptoDrive(RelicRecoveryVuMark vuMark, Direction direction, int timeoutS){
        switch (vuMark){
            case LEFT:
                encoderDriveIMU(.7, 30, direction, 10);
            case RIGHT:
                encoderDriveIMU(.7, 30, direction, 10);
            case CENTER:
                encoderDriveIMU(.7, 30, direction, 10);

        }
    }


    public void driveLefttoCrypto(RelicRecoveryVuMark vuMark, ColorSensor colors){
        if(vuMark == RelicRecoveryVuMark.RIGHT){
            moveAtSpeed(Direction.LEFT, .4);


        }
        moveAtSpeed(Direction.FORWARD, .4);

    }
    public void driveRighttoCrypto(RelicRecoveryVuMark vuMark, ColorSensor colors){
        if(vuMark == RelicRecoveryVuMark.RIGHT){
            moveAtSpeed(Direction.BACKWARD, .4);
            //stopOnRed(colors, );


        }
        moveAtSpeed(Direction.FORWARD, .4);

    }
    public void stopOnRed(ColorSensor colors, RelicRecoveryVuMark vuMark){
        int counterRed = 0;
        while (opMode.opModeIsActive()){
            if (detectRed(colors)){
                 setAllMotorSpeed(0);
            }

        }


    }

    public void rotateCCW(double power) {
        lF.setPower(-power);
        lB.setPower(-power);
        rF.setPower(power);
        rB.setPower(power);
    }

    public void rotateCW(double power) {
        lF.setPower(power);
        lB.setPower(power);
        rF.setPower(-power);
        rB.setPower(-power);
    }

    public void strafeLeft(double power) {
        lF.setPower(power);
        lB.setPower(-power);
        rF.setPower(-power);
        rB.setPower(power);
    }

    public void strafeRight(double power) {

        lF.setPower(-power);
        lB.setPower(power);
        rF.setPower(power);
        rB.setPower(-power);
    }

    public void resetEncoders() {
        this.lF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveFwd(double speed, double inches, double timeoutS) {
        resetEncoders();
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newLBTarget;
        int newRBTarget;
        int newRFTarget;
        int newLFTarget;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLBTarget = lB.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
            newRBTarget = rB.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
            newLFTarget = lF.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
            newRFTarget = rF.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
            lF.setTargetPosition(newLFTarget);
            rF.setTargetPosition(newRFTarget);
            lB.setTargetPosition(newLBTarget);
            rB.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
        timer.reset();
            lF.setPower(Math.abs(speed));
            rF.setPower(Math.abs(speed));
            lB.setPower(Math.abs(speed));
            rB.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (timer.time()< timeoutS*1000) &&
                    (lF.isBusy() && rF.isBusy() && rB.isBusy() && lB.isBusy())) {
            }

            // Stop all motion
            this.stopAll();

            // Turn off RUN_TO_POSITION
            lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // resetEncoders();
            //  sleep(250);   // optional pause after each move
        }
    public void slidesUpEncoder(double speed, double inches, double timeoutS) {
        resetEncoders();

        this.rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newRSTarget;
        int newLSTarget;

        newRSTarget = (int) (inches * TICKS_PER_INCH_FORWARD);
        newLSTarget = (int) (inches * TICKS_PER_INCH_FORWARD);

        rSlide.setTargetPosition(newRSTarget);
        lSlide.setTargetPosition(newLSTarget);

        rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timer.reset();
        while (opMode.opModeIsActive() &&
                (timer.time()< timeoutS*1000) &&
                (rSlide.isBusy() && lSlide.isBusy())) {

        }

        rSlide.setPower(0);
        lSlide.setPower(0);

        this.rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveSlidesTime(double speed, int timeMS){
        slidesSetPower(speed);
        Functions.waitFor(timeMS);
        slidesSetPower(0);
    }

    public void encoderDrive(double speed, double inches, Direction direction, double timeoutS) {
        resetEncoders();
        this.lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newLBTarget;
        int newRBTarget;
        int newRFTarget;
        int newLFTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        switch (direction) {
            case RIGHT:
                newLBTarget = lB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            case LEFT:
                newLBTarget = lB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            case FORWARD:
                newLBTarget = lB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            case BACKWARD:
                newLBTarget = lB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            default:
                newLBTarget = lB.getCurrentPosition();
                newRBTarget = rB.getCurrentPosition();
                newLFTarget = lF.getCurrentPosition();
                newRFTarget = rF.getCurrentPosition();
                break;
        }
        lF.setTargetPosition(newLFTarget);
        rF.setTargetPosition(newRFTarget);
        lB.setTargetPosition(newLBTarget);
        rB.setTargetPosition(newRBTarget);

        // Turn On RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timer.reset();
        switch (direction) {
            case RIGHT:
                lF.setPower(-Math.abs(speed));
                rF.setPower(Math.abs(speed));
                lB.setPower(Math.abs(speed));
                rB.setPower(-Math.abs(speed));
                break;
            case LEFT:
                lF.setPower(Math.abs(speed));
                rF.setPower(-Math.abs(speed));
                lB.setPower(-Math.abs(speed));
                rB.setPower(Math.abs(speed));
                break;
            case FORWARD:
                lF.setPower(Math.abs(speed));
                rF.setPower(Math.abs(speed));
                lB.setPower(Math.abs(speed));
                rB.setPower(Math.abs(speed));
                break;
            case BACKWARD:
                lF.setPower(-Math.abs(speed));
                rF.setPower(-Math.abs(speed));
                lB.setPower(-Math.abs(speed));
                rB.setPower(-Math.abs(speed));
                break;
            default:
                lF.setPower(0);
                rF.setPower(0);
                lB.setPower(0);
                rB.setPower(0);
                break;
        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opMode.opModeIsActive() &&
                (timer.time()< timeoutS*1000) &&
                (lF.isBusy() && rF.isBusy() && rB.isBusy() && lB.isBusy())) {

        }

        // Stop all motion
        this.stopAll();

        // Turn off RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // resetEncoders();
        //  sleep(250);   // optional pause after each move
    }

    public void encoderDriveIMU(double speed, double inches, Direction direction, double timeoutS) {
        resetEncoders();
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newLBTarget;
        int newRBTarget;
        int newRFTarget;
        int newLFTarget;

        // Get current heading (should be maintained throughout drive)
        double targetHeading = imu.getAngle();
        double e;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        switch (direction) {
            case RIGHT:
                newLBTarget = lB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            case LEFT:
                newLBTarget = lB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            case FORWARD:
                newLBTarget = lB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() + (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            case BACKWARD:
                newLBTarget = lB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRBTarget = rB.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newLFTarget = lF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                newRFTarget = rF.getCurrentPosition() - (int) (inches * TICKS_PER_INCH_FORWARD);
                break;
            default:
                newLBTarget = lB.getCurrentPosition();
                newRBTarget = rB.getCurrentPosition();
                newLFTarget = lF.getCurrentPosition();
                newRFTarget = rF.getCurrentPosition();
                break;
        }
        lF.setTargetPosition(newLFTarget);
        rF.setTargetPosition(newRFTarget);
        lB.setTargetPosition(newLBTarget);
        rB.setTargetPosition(newRBTarget);

        // Turn On RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timer.reset();
        switch (direction) {
            case RIGHT:
                lF.setPower(-Math.abs(speed));
                rF.setPower(Math.abs(speed));
                lB.setPower(Math.abs(speed));
                rB.setPower(-Math.abs(speed));
                break;
            case LEFT:
                lF.setPower(Math.abs(speed));
                rF.setPower(-Math.abs(speed));
                lB.setPower(-Math.abs(speed));
                rB.setPower(Math.abs(speed));
                break;
            case FORWARD:
                lF.setPower(Math.abs(speed));
                rF.setPower(Math.abs(speed));
                lB.setPower(Math.abs(speed));
                rB.setPower(Math.abs(speed));
                break;
            case BACKWARD:
                lF.setPower(-Math.abs(speed));
                rF.setPower(-Math.abs(speed));
                lB.setPower(-Math.abs(speed));
                rB.setPower(-Math.abs(speed));
                break;
            default:
                lF.setPower(0);
                rF.setPower(0);
                lB.setPower(0);
                rB.setPower(0);
                break;
        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opMode.opModeIsActive() &&
                (timer.time()< timeoutS*1000) &&
                (lF.isBusy() && rF.isBusy() && rB.isBusy() && lB.isBusy())) {
            e = targetHeading - imu.getAngle();

            if (Math.abs(e) > driveStraightErrorMargin) {

                switch (direction) {
                    case RIGHT:
                        if(e > 0) { //CW
                            lF.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rF.setPower((1 + powerAdjust(Math.abs(e))) * Math.abs(speed));
                            lB.setPower((1 + powerAdjust(Math.abs(e))) * Math.abs(speed));
                            rB.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));

                        } else { // CCW
                            lF.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rF.setPower((1 - powerAdjust(Math.abs(e))) * Math.abs(speed));
                            lB.setPower((1 - powerAdjust(Math.abs(e))) * Math.abs(speed));
                            rB.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                        }
                        break;
                    case LEFT:
                        if(e > 0) { //CW
                            lF.setPower((1 - powerAdjust(Math.abs(e))) * (Math.abs(speed)));
                            rF.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            lB.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rB.setPower((1 - powerAdjust(Math.abs(e))) * (Math.abs(speed)));

                        } else { // CCW
                            lF.setPower((1 + powerAdjust(Math.abs(e))) * (Math.abs(speed)));
                            rF.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            lB.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rB.setPower((1 + powerAdjust(Math.abs(e))) * (Math.abs(speed)));
                        }
                        break;
                    case FORWARD:
                        if(e > 0) { //CW
                            lF.setPower((1 - powerAdjust(Math.abs(e))) * Math.abs(speed));
                            rF.setPower((1 + powerAdjust(Math.abs(e))) * Math.abs(speed));
                            lB.setPower((1 - powerAdjust(Math.abs(e))) * Math.abs(speed));
                            rB.setPower((1 + powerAdjust(Math.abs(e))) * Math.abs(speed));

                        } else { // CCW
                            lF.setPower((1 + powerAdjust(Math.abs(e))) * Math.abs(speed));
                            rF.setPower((1 - powerAdjust(Math.abs(e))) * Math.abs(speed));
                            lB.setPower((1 + powerAdjust(Math.abs(e))) * Math.abs(speed));
                            rB.setPower((1 - powerAdjust(Math.abs(e))) * Math.abs(speed));
                        }
                        break;
                    case BACKWARD:
                        if(e > 0) { //CW
                            lF.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rF.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            lB.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rB.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));

                        } else { // CCW
                            lF.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rF.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            lB.setPower((1 + powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                            rB.setPower((1 - powerAdjust(Math.abs(e))) * (-Math.abs(speed)));
                        }
                        break;
                    default:
                        lF.setPower(0);
                        rF.setPower(0);
                        lB.setPower(0);
                        rB.setPower(0);
                        break;
                }

            }
        }

        // Stop all motion
        this.stopAll();

        // Turn off RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // resetEncoders();
        //  sleep(250);   // optional pause after each move
    }

    public void moveBkwd(double speed, double inches, double timeoutS) {
        resetEncoders();
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newLBTarget;
        int newRBTarget;
        int newRFTarget;
        int newLFTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLBTarget = lB.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        newRBTarget = rB.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        newLFTarget = lF.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        newRFTarget = rF.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        lF.setTargetPosition(newLFTarget);
        rF.setTargetPosition(newRFTarget);
        lB.setTargetPosition(newLBTarget);
        rB.setTargetPosition(newRBTarget);

        // Turn On RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timer.reset();
        lF.setPower(Math.abs(-speed));
        rF.setPower(Math.abs(-speed));
        lB.setPower(Math.abs(-speed));
        rB.setPower(Math.abs(-speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opMode.opModeIsActive() &&
                (timer.time()< timeoutS*1000) &&
                (lF.isBusy() && rF.isBusy() && rB.isBusy() && lB.isBusy())) {
        }

        // Stop all motion
        this.stopAll();

        // Turn off RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // resetEncoders();
        //  sleep(250);   // optional pause after each move
    }

    public void stopAll() {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }

    public void setDriveMotorMode ( DcMotor.RunMode mode) {
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveLeft(double speed, double inches, double timeoutS) {
        resetEncoders();
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newLBTarget;
        int newRBTarget;
        int newRFTarget;
        int newLFTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLBTarget = lB.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        newRBTarget = rB.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
        newLFTarget = lF.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
        newRFTarget = rF.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        lF.setTargetPosition(newLFTarget);
        rF.setTargetPosition(newRFTarget);
        lB.setTargetPosition(newLBTarget);
        rB.setTargetPosition(newRBTarget);

        // Turn On RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timer.reset();
        lF.setPower(Math.abs(speed));
        rF.setPower(Math.abs(speed));
        lB.setPower(Math.abs(speed));
        rB.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opMode.opModeIsActive() &&
                (timer.time()< timeoutS*1000) &&
                (lF.isBusy() && rF.isBusy() && rB.isBusy() && lB.isBusy())) {
        }

        // Stop all motion
        this.stopAll();

        // Turn off RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void selfBalance(Telemetry telemetry) {
        // call when the robot is fully on the platform
        /* also assumes that after 0 degrees to the right starts incrementing positively
         * and before 0 degrees to the left is incrementing negatively
        */
        //using trig you know that the robot will be stable if the angle is less than 10 degrees. Thus, we will have a threshold of 5
        boolean pitchDone = false;
        boolean rollDone = false;
        int checkTimeMS = 1000;

        while (opMode.opModeIsActive()) {
            double pitch = imu.getOrientation()[1];
            double roll = imu.getOrientation()[2];
            telemetry.addData("Initial Pitch", pitch);
            telemetry.addData("Initial Roll", roll);

            //initialized as false to check again
            while ((!pitchDone || !rollDone)&& opMode.opModeIsActive()) {
                pitch = imu.getOrientation()[1];
                roll = imu.getOrientation()[2];
                telemetry.addData("Pitch", pitch);
                telemetry.addData("Roll", roll);
                if (pitch > balanceThreshold && !pitchDone) {
                    moveAtSpeed(Direction.BACKWARD, powerPerDegree(pitch));
                } else if (pitch < -balanceThreshold) {
                    moveAtSpeed(Direction.FORWARD, powerPerDegree(pitch));
                } else {
                    pitchDone = true;
                    stopAll();
                    telemetry.addLine("Pitch Done");
                }

                if (roll > balanceThreshold && !rollDone) {
                    moveAtSpeed(Direction.LEFT, powerPerDegree(roll));
                } else if (roll < -balanceThreshold) {
                    moveAtSpeed(Direction.RIGHT, powerPerDegree(roll));
                } else {
                    rollDone = true;
                    telemetry.addLine("Roll Done");
                    stopAll();
                }
                telemetry.update();
                //Functions.waitFor(50);
            }
            Functions.waitFor(checkTimeMS);
            //CHECK TO SEE IF OVERSHOT AND IS STILL WITHIN THRESHOLD
            if(Math.abs(pitch)<balanceThreshold && Math.abs(roll)<balanceThreshold){
                telemetry.addLine("Overall Done");
                telemetry.update();
                stopAll();
                break;
            }
            pitchDone=false;
            rollDone=false;
        }

    }

    public double powerPerDegree(double degree) {
        return ((degree-balanceThreshold)*balanceMultiplier)+minMotorPower;
        //proportional speed, minimum movement speed at threshold

    }

    public void setAllMotorSpeed(double speed){
        lF.setPower((speed));
        rF.setPower((speed));
        lB.setPower((speed));
        rB.setPower((speed));
    }

    public void moveRight(double speed, double inches, double timeoutS) {
        resetEncoders();
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newLBTarget;
        int newRBTarget;
        int newRFTarget;
        int newLFTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLBTarget = lB.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
        newRBTarget = rB.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        newLFTarget = lF.getCurrentPosition() - (int)(inches * TICKS_PER_INCH_FORWARD);
        newRFTarget = rF.getCurrentPosition() + (int)(inches * TICKS_PER_INCH_FORWARD);
        lF.setTargetPosition(newLFTarget);
        rF.setTargetPosition(newRFTarget);
        lB.setTargetPosition(newLBTarget);
        rB.setTargetPosition(newRBTarget);

        // Turn On RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timer.reset();
        lF.setPower(Math.abs(speed));
        rF.setPower(Math.abs(speed));
        lB.setPower(Math.abs(speed));
        rB.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opMode.opModeIsActive() &&
                (timer.time()< timeoutS*1000) &&
                (lF.isBusy() && rF.isBusy() && rB.isBusy() && lB.isBusy())) {
        }

        // Stop all motion
        this.stopAll();

        // Turn off RUN_TO_POSITION
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }


    public void debugColor(ColorSensor colorsensor, Telemetry telemetry) {
        while (System.currentTimeMillis()
                < System.currentTimeMillis() + 10000000000L && opMode.opModeIsActive()) {
            telemetry.addData("Red", colorsensor.red());
            telemetry.addData("Green", colorsensor.green());
            telemetry.addData("Blue", colorsensor.blue());
            telemetry.update();
            Functions.waitFor(50);
        }
    }

    public boolean detectWhite(ColorSensor colorsensor) {
        int red = colorsensor.red();
        int green = colorsensor.green();
        int blue = colorsensor.blue();
        average = (red + green + blue) / 3;
        return (average > 17);
    }

    public void detectAmbientLight(ColorSensor colorsensor){
        ambientBlue = colorsensor.blue();
        ambientRed = colorsensor.red();
    }
    public boolean detectColorAmbient(char color, ColorSensor colorsensor){
        if(color == 'b')
            return colorsensor.blue() - ambientBlue > Functions.blueAmbientThreshold;
        else if(color == 'r')
            return colorsensor.red() - ambientRed > Functions.redAmbientThreshold;
        else return false;
    }

    public boolean detectRed(ColorSensor colorsensor) {
        int red = colorsensor.red();
        int blue = colorsensor.blue();
        return red>Functions.redThreshold;
    }

}
