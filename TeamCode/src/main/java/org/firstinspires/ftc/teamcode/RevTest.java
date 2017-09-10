
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RevTest")
public class RevTest extends LinearOpMode {
    static DcMotor motor1;
    //static Servo servo1;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(1);
            Functions.waitFor(3000);
            motor1.setPower(0);


        }
    }

    public void initHardware() {

        motor1 = hardwareMap.dcMotor.get("motor1");

        //servo1 = hardwareMap.servo.get("servo1");

    }

}

