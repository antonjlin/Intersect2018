package org.firstinspires.ftc.teamcode.tasks;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.Functions;

public class FlipTask extends TaskThread {

    private Servo lFlip,rFlip;

    double flipDownPosR = 0;
    double flipUpPosR = .6;
    double flipInterPosR = 0.1;
    double flipDownPosL = 0;
    double flipUpPosL = .9;
    double flipInterPosL = 0.4;

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
            if(opMode.gamepad1.dpad_right){
                pos--;
            }
            if (opMode.gamepad1.dpad_left) {
                  pos++;
            }
            if(pos == -1){
                pos = 0;
            } else if (pos == 0){
                setFlipPosDown();
            } else if(pos == 1){
                setFlipPosInter();
            } else if(pos == 2){
                setFlipPosUp();
            } else if(pos == 3){
                pos = 2;
            }

            Functions.waitFor(200);
            opMode.telemetry.addData("position: ",pos);
            opMode.telemetry.update();

        }


    }

    public void setFlipPosDown() {
        lFlip.setPosition(flipDownPosL);
        rFlip.setPosition(flipDownPosR);
    }
    public void setFlipPosInter() {
        lFlip.setPosition(flipInterPosL);
        rFlip.setPosition(flipInterPosR);
    }
    public void setFlipPosUp(){
        lFlip.setPosition(flipUpPosL);
        rFlip.setPosition(flipUpPosR);

    }
    @Override
    public void initialize() {
        lFlip = opMode.hardwareMap.servo.get("lFlip");
        rFlip = opMode.hardwareMap.servo.get("rFlip");
        lFlip.setDirection(Servo.Direction.REVERSE);
        rFlip.setDirection(Servo.Direction.REVERSE);
        setFlipPosDown();
    }
}
