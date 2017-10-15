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
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.IMU;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleopnew")
public class TeleOpNew extends LinearOpMode {
    static DcMotor rFmotor, rBmotor, lFmotor, lBmotor;
    DriveTrain driveTrain;
    static GyroSensor gyro;
    static ColorSensor floorColor;
    static DcMotor rightConv, leftConv, leftSlide, rightSlide;
    private BNO055IMU adaImu;
    private IMU imu;
    int leftSlidePos;
    int rightSlidePos;
    int slideTicksPerInch;
    int pos0 = 0;
    int pos1 = 6;
    int pos2 = 12;
    int pos3 = 18;
    int pos4 = 24;



    // RampFlywheel rampFlywheel = new RampFlywheel();
    // RampDownFlywheel rampDownFlywheel = new RampDownFlywheel();
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            //DRIVETRAIN FUNCTIONS
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double frontLeft = r * Math.cos(robotAngle) + rightX;
            final double frontRight = r * Math.sin(robotAngle) - rightX;
            final double backLeft = r * Math.sin(robotAngle) + rightX;
            final double backRight = r * Math.cos(robotAngle) - rightX;

            // for intake and placing glyphs
            //  NEED TO CHANGE BUTTONS!!!
            if (gamepad1.b) {
                driveTrain.rollersSetPower(0.2);
            }

            // for opposite direction just incase
            //  NEED TO CHANGE BUTTONS!!!
            if (gamepad1.x) {
                driveTrain.rollersSetPower(-0.2);
            }

            if (gamepad1.right_bumper) {
                driveTrain.slidesSetPower(0.2);
            }

            if (gamepad1.left_bumper) {
                driveTrain.slidesSetPower(-0.2);
            }
            /*
            double currentPos = rightSlide.getCurrentPosition();
            if (gamepad1.y) {
                if (currentPos < 5.75) {
                    driveTrain.encoderMoveSlides(0.2, 6.25, 10000);
                } else if (currentPos < 11.75) {
                    driveTrain.encoderMoveSlides(0.2, 12.25, 10000);
                } else if (currentPos < 17.75) {
                    driveTrain.encoderMoveSlides(0.2, 18.25, 10000);
                }
            }
            if (gamepad1.a) {
                if (currentPos > 12.75) {
                    driveTrain.encoderMoveSlides(0.2, 12.25, 10000);
                } else if (currentPos > 6.75) {
                    driveTrain.encoderMoveSlides(0.2, 6.25, 10000);
                } else if (currentPos > 0.25) {
                    driveTrain.encoderMoveSlides(0.2, 0, 10000);
                }
            }
            */

            lFmotor.setPower(frontLeft);
            rFmotor.setPower(frontRight);
            lBmotor.setPower(backLeft);
            rBmotor.setPower(backRight);

            if(gamepad1.a){
                driveTrain.selfBalance(telemetry);
            }

            telemetry.addData("lf", frontLeft);
            telemetry.addData("rf", frontRight);
            telemetry.addData("lb", backLeft);
            telemetry.addData("rb", backRight);
            telemetry.update();
            }
        }
        public void initHardware(){
            driveTrain = new DriveTrain(this);
            rightConv = hardwareMap.dcMotor.get("rightConv");
            leftConv = hardwareMap.dcMotor.get("leftConv");
            rFmotor = hardwareMap.dcMotor.get("rF");
            rightConv = hardwareMap.dcMotor.get("rightSlide");
            leftConv = hardwareMap.dcMotor.get("leftSlide");
            rBmotor = hardwareMap.dcMotor.get("rB");
            lFmotor = hardwareMap.dcMotor.get("lF");
            lBmotor = hardwareMap.dcMotor.get("lB");

            lBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightConv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftConv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightConv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftConv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            lBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
            lFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightConv.setDirection(DcMotorSimple.Direction.REVERSE);
            leftConv.setDirection(DcMotorSimple.Direction.FORWARD);
            leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            adaImu = hardwareMap.get(BNO055IMU.class, "IMU");
            imu = new IMU(adaImu);


        }

}