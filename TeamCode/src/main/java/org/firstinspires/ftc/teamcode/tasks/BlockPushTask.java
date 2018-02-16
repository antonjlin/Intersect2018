package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.Functions;

/**
 * Created by pranav on 2/2/18.
 */

public class BlockPushTask extends TaskThread{

    private Servo blockPush;
//    private DigitalChannel cryptoTouch;
    private String touchState;
    private Servo touchServoRight;
    double touchDownPos = .7;
    double touchUpPos = .3;


    int pos = 0;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public BlockPushTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void run() {
        timer.reset();
        while (opMode.opModeIsActive() && running) {
            if(opMode.gamepad2.b || opMode.gamepad1.b){
                blockPush.setPosition(0);
            }
            if (opMode.gamepad2.a || opMode.gamepad1.a){
                blockPush.setPosition(.7);
            }
            Functions.waitFor(100);
            opMode.telemetry.addData("position: ",pos);
//            touchState = String.valueOf(cryptoTouch.getState());
            opMode.telemetry.addData("TouchSensor", touchState);
            opMode.telemetry.update();
        }
    }

    @Override
    public void initialize() {
        blockPush = opMode.hardwareMap.servo.get("blockPusher");
        blockPush.setDirection(Servo.Direction.FORWARD);
        blockPush.setPosition(.7);
        touchServoRight = opMode.hardwareMap.servo.get("touchServoRight");
        touchServoRight.setPosition(touchUpPos);
//        cryptoTouch  = opMode.hardwareMap.get(DigitalChannel.class, "cryptoTouch");

    }


}
