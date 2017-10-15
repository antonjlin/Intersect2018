package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 10/15/16.
 */
public class SlideTask extends TaskThread {

    private DcMotor lSlide , rSlide;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public SlideTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void run() {
        timer.reset();
        while (opMode.opModeIsActive() && running) {
            if (opMode.gamepad1.right_bumper) {
                setSlidePower(1);
            } else if (opMode.gamepad1.left_bumper) {
                setSlidePower(-1);
            } else{
                setSlidePower(0);
            }
        }
    }

    public void setSlidePower(double power) {
        rSlide.setPower(power);
        lSlide.setPower(power);
    }
    @Override
    public void initialize() {
        lSlide = opMode.hardwareMap.dcMotor.get("lSlide");
        rSlide = opMode.hardwareMap.dcMotor.get("rSlide");
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        lSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
