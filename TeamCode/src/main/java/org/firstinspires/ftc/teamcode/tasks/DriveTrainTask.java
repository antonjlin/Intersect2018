package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 10/15/16.
 */
public class DriveTrainTask extends TaskThread {

    private DcMotor lF, rF, lB, rB;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double zeroAngle, joyStickAngle, gyroAngle;

    public DriveTrainTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void run() {
        timer.reset();
        while (opMode.opModeIsActive() && running) {
            double r = Math.hypot(opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
            double robotAngle = Math.atan2(opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -opMode.gamepad1.right_stick_x;
            final double frontLeft = r * Math.cos(robotAngle) + rightX;
            final double frontRight = r * Math.sin(robotAngle) - rightX;
            final double backLeft = r * Math.sin(robotAngle) + rightX;
            final double backRight = r * Math.cos(robotAngle) - rightX;

            lF.setPower(frontLeft);
            rF.setPower(frontRight);
            lB.setPower(backLeft);
            rB.setPower(backRight);

        }
    }
    @Override
    public void initialize() {
        lF = opMode.hardwareMap.dcMotor.get("lF");
        rF = opMode.hardwareMap.dcMotor.get("rF");
        lB = opMode.hardwareMap.dcMotor.get("lB");
        rB = opMode.hardwareMap.dcMotor.get("rB");

        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
