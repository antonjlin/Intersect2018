/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop")
public class TeleOpCurved extends LinearOpMode {
    static DcMotor rFmotor, rBmotor, lFmotor, lBmotor, flywheel1, flywheel2, sweeperLow, liftMotor ;
    static Servo lLift, rLift;
    public static boolean flywheelUpPressed = false;
    public static boolean flywheelDownPressed = false;

    // RampFlywheel rampFlywheel = new RampFlywheel();
    // RampDownFlywheel rampDownFlywheel = new RampDownFlywheel();
    public void runOpMode() throws InterruptedException {
        double flywheelSpeed = 0;
        double fwdPower, strafePower, rotationPower;

        boolean fwdSweeperLow, bkwdSweeperLow, fwdSweeperLow2, bkwdSweeperLow2 , toggleWall, wallUp = true, wallControl = false;
        boolean slideUp, slideDown, liftUp, liftDown, gateClose, extendPusher, retractPusher,
                decreaseFlywheel, increaseFlywheel, pushBeacon;
        CRServo buttonPusher = hardwareMap.crservo.get("buttonPusher");
        rFmotor = hardwareMap.dcMotor.get("rF");
        rBmotor = hardwareMap.dcMotor.get("rB");
        lFmotor = hardwareMap.dcMotor.get("lF");
        lBmotor = hardwareMap.dcMotor.get("lB");
        flywheel1 = hardwareMap.dcMotor.get("flywheel1");
        flywheel2 = hardwareMap.dcMotor.get("flywheel2");
        sweeperLow = hardwareMap.dcMotor.get("sweeperLow");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        Servo sideWall = hardwareMap.servo.get("sideWall");
        Servo lLift = hardwareMap.servo.get("lLift");
        Servo rLift = hardwareMap.servo.get("rLift");
        rLift.setDirection(Servo.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperLow.setDirection(DcMotorSimple.Direction.REVERSE);
        sideWall.setPosition(Functions.sideWallUpPos);
        waitForStart();

        while (opModeIsActive()) {
            fwdPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            rotationPower = gamepad1.right_stick_x;
            fwdSweeperLow = gamepad1.a;
            fwdSweeperLow2 =  gamepad2.b;
            bkwdSweeperLow = gamepad1.b;
            bkwdSweeperLow2 = gamepad2.a;
            liftDown = gamepad1.dpad_down || gamepad2.dpad_down;
            liftUp = gamepad1.dpad_up || gamepad2.dpad_up;
            increaseFlywheel = gamepad2.right_bumper || gamepad1.right_bumper;
            decreaseFlywheel = gamepad2.left_bumper || gamepad1.left_bumper;
            extendPusher = gamepad1.y || gamepad2.y;
            retractPusher = gamepad1.x || gamepad2.x;
            toggleWall = gamepad2.start || gamepad1.back;


            if (gamepad1.start) {
                lFmotor.setPower((fwdPower + strafePower + rotationPower) / 2);
                lBmotor.setPower((fwdPower - strafePower + rotationPower) / 2);
                rFmotor.setPower((fwdPower - strafePower - rotationPower) / 2);
                rBmotor.setPower((fwdPower + strafePower - rotationPower) / 2);
            } else {
                lFmotor.setPower(fwdPower + strafePower + rotationPower);
                lBmotor.setPower(fwdPower - strafePower + rotationPower);
                rFmotor.setPower(fwdPower - strafePower - rotationPower);
                rBmotor.setPower(fwdPower + strafePower - rotationPower);
            }
            if(toggleWall && !wallUp && !wallControl){
                sideWall.setPosition(Functions.sideWallUpPos);
                wallUp = true;
                wallControl = true;
            }
            if(toggleWall && wallUp && !wallControl){
                sideWall.setPosition(Functions.sideWallDownPos);
                wallUp = false;
                wallControl = true;
            }
            if(!toggleWall){
                wallControl = false;
            }
            if (extendPusher) {
                buttonPusher.setPower(1);
            }
            if (retractPusher) {
                buttonPusher.setPower(-1);
            } else {
                buttonPusher.setPower(0);
            }

            if (increaseFlywheel && !flywheelUpPressed) {
                flywheelUpPressed = true;
                if (flywheelSpeed == -.3) {
                    flywheelSpeed = 0;
                    flywheel1.setPower(flywheelSpeed);
                    flywheel2.setPower(flywheelSpeed);
                } else if (flywheelSpeed == 0) {
                    sweeperLow.setPower(-1);
                    Functions.waitFor(150);
                    sweeperLow.setPower(0);
                    flywheelSpeed = .6;
                    flywheel1.setPower(flywheelSpeed);
                    flywheel2.setPower(flywheelSpeed);
                } else  if (flywheelSpeed == .6) {
                    flywheelSpeed = 1;
                    flywheel1.setPower(flywheelSpeed);
                    flywheel2.setPower(flywheelSpeed);
                }
            }
            if (decreaseFlywheel && !flywheelDownPressed) {
                flywheelDownPressed = true;
                if (flywheelSpeed == 1) {
                    flywheelSpeed = 0.6;
                    flywheel1.setPower(flywheelSpeed);
                    flywheel2.setPower(flywheelSpeed);
                } else if (flywheelSpeed == 0.6) {
                    flywheelSpeed = 0;
                    flywheel1.setPower(flywheelSpeed);
                    flywheel2.setPower(flywheelSpeed);
                } else if (flywheelSpeed == 0) {
                    flywheelSpeed = -.3;
                    flywheel1.setPower(flywheelSpeed);
                    flywheel2.setPower(flywheelSpeed);
                }
            }
            if(!increaseFlywheel){
                flywheelUpPressed = false;
            }
            if(!decreaseFlywheel){
                flywheelDownPressed = false;
            }

            if (fwdSweeperLow) {
                sweeperLow.setPower(1);
            } else if (bkwdSweeperLow) {
                sweeperLow.setPower(-1);
            } else if (fwdSweeperLow2) {
                sweeperLow.setPower(1);
            } else if (bkwdSweeperLow2) {
                flywheelSpeed = 0;
                sweeperLow.setPower(-1);
            } else {
                sweeperLow.setPower(0);
            }


            if (gamepad1.left_trigger > 0.1)
                liftMotor.setPower(-gamepad1.left_trigger);
            else if (gamepad2.left_trigger > 0.1)
                liftMotor.setPower(-gamepad2.left_trigger);
            else if (gamepad1.right_trigger > 0.1)
                liftMotor.setPower(gamepad1.right_trigger);
            else if (gamepad2.right_trigger > 0.1)
                liftMotor.setPower(gamepad2.right_trigger);
            else {
                liftMotor.setPower(0);
            }
            if (liftDown) {
                rLift.setPosition(Functions.liftDownPos);
                lLift.setPosition(Functions.liftDownPos);
            }
            if (liftUp) {
                rLift.setPosition(Functions.liftUpPos);
                lLift.setPosition(Functions.liftUpPos);
            }


        }
    }


}