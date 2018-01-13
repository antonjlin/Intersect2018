package org.firstinspires.ftc.teamcode.tasks;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlipTask extends TaskThread {

    private Servo lFlip,rFlip;

    double flipDownPos = 0;
    double flipUpPos = .6;
    double flipInterPos = 0.1;
    int pos = 0;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public FlipTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void run() {
        timer.reset();
        while (opMode.opModeIsActive() && running) {
            if (opMode.gamepad1.dpad_left) {
                if (pos == 0) {

                } else if (pos == 1) {
                    setFlipPos(flipDownPos);
                    pos = 0;
                } else if (pos == 2) {
                    setFlipPos(flipInterPos);
                    pos = 1;
                }
            } else if(opMode.gamepad1.dpad_right){
                if (pos == 0) {
                    setFlipPos(flipInterPos);
                    pos = 1;
                } else if (pos == 1) {
                    setFlipPos(flipUpPos);
                    pos = 2;
                } else if (pos == 2) {
                }
            }
        }
    }

    public void setFlipPos(double position) {
        lFlip.setPosition(position);
        rFlip.setPosition(position);
    }
    @Override
    public void initialize() {
        lFlip = opMode.hardwareMap.servo.get("lFlip");
        rFlip = opMode.hardwareMap.servo.get("rFlip");
        lFlip.setDirection(Servo.Direction.FORWARD);
        rFlip.setDirection(Servo.Direction.REVERSE);
    }
}
