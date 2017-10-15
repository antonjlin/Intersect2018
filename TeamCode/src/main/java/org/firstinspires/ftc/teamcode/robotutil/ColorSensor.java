package org.firstinspires.ftc.teamcode.robotutil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ColorSensorFront.java
 * Class created to encapsulate all Color Sensor
 * functionality and output. Includes averaging multiple readings
 * for accurate output.
 *
 * Created by Victor on 10/14/2017.
 */


public class ColorSensor {

    ColorSensor sensor;
    private ElapsedTime logMessageTimer;
    LinearOpMode opMode;
    public Team team = Team.BLUE;
    public boolean lightOn = false;

    public ColorSensor(ColorSensor sensor, LinearOpMode opMode) {
        this.opMode = opMode;
        this.sensor = sensor;
        logMessageTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        logMessageTimer.reset();
    }

    public void enableLED(boolean b)
    {
        sensor.enableLED(b);
    }

    public int getRed()
    {
        return sensor.getRed();
    }

    public int getBlue()
    {
        return sensor.getBlue();
    }

    public int getGreen() {
        return sensor.getGreen();
    }

    public int getAlpha()
    {
        return sensor.getAlpha();
    }

    public void debugOutput(String output) {
        if(logMessageTimer.time() > 50) {
            logMessageTimer.reset();
            System.out.println(output);
        }
    }

    public boolean isWhite() {
        int red =  getRed();
        int blue = getBlue();
        int green = getGreen();
        int i = 0;
        int score = 0;
        logMessageTimer.reset();
        while (i < 10 && opMode.opModeIsActive()){
            if (logMessageTimer.time() > 3){
                i++;
                System.out.println("R: " + sensor.getRed() + " G: " + sensor.getGreen()+ " B: "  + sensor.getBlue() );
                logMessageTimer.reset();
                if (sensor.getRed() >= 3 && sensor.getBlue() >= 3 && sensor.getGreen() >= 3)
                    score ++;
            }
        }
        return score >= 3;
    }

    public boolean isBlue() {
        int i = 0;
        int score = 0;
        int margin = 4; // 20
        if (lightOn) {
            margin = 10;
        }
        logMessageTimer.reset();
        while (i < 10 && opMode.opModeIsActive()) {
            if (logMessageTimer.time() > 3){
                i++;
                logMessageTimer.reset();
                if (sensor.getBlue() - sensor.getRed() >= margin) {
                    score++;
                }
            }
        }
        return score >= 5;
    }

    public boolean isRed() {
        int i = 0;
        int score = 0;
        logMessageTimer.reset();
        int margin = 9; // 19
        if (lightOn) {
            margin = 10;
        }
        while (i < 10 && opMode.opModeIsActive()) {
            if (logMessageTimer.time() > 3){
                i++;
                logMessageTimer.reset();
                if (sensor.getRed() - sensor.getBlue() > margin) {
                    score++;
                }
            }
        }
        return score >= 4;
    }

    public boolean correctColor() {
        if (team == Team.BLUE) {
            return isBlue();
        }
        return isRed();
    }

    public boolean wrongColor() {
        if (team == Team.BLUE) {
            return isRed();
        }
        return isBlue();
    }

    public String getColor() {
        return "R: " + getRed() + " G: " + getGreen() + " B: " + getBlue();
    }
}