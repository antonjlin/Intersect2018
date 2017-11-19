
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Functions;
import org.firstinspires.ftc.teamcode.robotutil.IMU;
import org.firstinspires.ftc.teamcode.robotutil.MRColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.Team;

@Autonomous(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {
    // ModernRoboticsI2cColorSensor jewelColor;
    ColorSensor jewelColor;
    //MRColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        //initHardware();
        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        jewelColor.enableLed(false);
        Functions.waitFor(100);
        jewelColor.enableLed(true);
        Functions.waitFor(100);
        jewelColor.enableLed(false);
        Functions.waitFor(100);
        jewelColor.enableLed(true);
        Functions.waitFor(100);
        jewelColor.enableLed(false);
        Functions.waitFor(100);
        jewelColor.enableLed(true);
        Functions.waitFor(100);
        jewelColor.enableLed(false);
        Functions.waitFor(100);
        jewelColor.enableLed(true);
        Functions.waitFor(100);
        waitForStart();
        while (opModeIsActive()) {
            //colorSensor.telemetryDebug(this);
            telemetry.addData("blue: ", jewelColor.blue());
            telemetry.addData("red: ", jewelColor.red());
            //colorSensor = new MRColorSensor(jewelColor, this);
            telemetry.update();
        }
    }
}
