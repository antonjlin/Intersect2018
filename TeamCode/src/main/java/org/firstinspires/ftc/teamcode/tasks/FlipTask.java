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
            if(opMode.gamepad1.dpad_right){
                pos--;
            }
            if (opMode.gamepad1.dpad_left) {
                  pos++;
            }
            if(pos == -1){
                pos = 0;
            } else if (pos == 0){
                setFlipPos(flipDownPos);
            } else if(pos == 1){
                setFlipPos(flipInterPos);
            } else if(pos == 2){
                setFlipPos(flipUpPos);
            } else if(pos == 3){
                pos = 2;
            }

            Functions.waitFor(200);
            opMode.telemetry.addData("position: ",pos);
            opMode.telemetry.update();

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
        setFlipPos(flipDownPos);
    }
}
