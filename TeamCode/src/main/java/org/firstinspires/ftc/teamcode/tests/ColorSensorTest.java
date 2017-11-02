
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.Team;

@Autonomous(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor jewelColor;
    MRColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while(opModeIsActive())
            colorSensor.telemetryDebug(this);
    }

    public void initHardware() {
        colorSensor = new MRColorSensor(jewelColor, this);
        colorSensor.enableLED(false);
        telemetry.addLine("Done");
        telemetry.update();
    }
}
