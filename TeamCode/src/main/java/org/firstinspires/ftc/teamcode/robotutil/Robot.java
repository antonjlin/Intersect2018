package org.firstinspires.ftc.teamcode.robotutil;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by antonlin on 4/24/18.
 */

public class Robot{
    Robot(OpMode opMode){
        DcMotor lF = opMode.hardwareMap.dcMotor.get("lF");
        DcMotor rF = opMode.hardwareMap.dcMotor.get("rF");
        DcMotor lB = opMode.hardwareMap.dcMotor.get("lB");
        DcMotor rB = opMode.hardwareMap.dcMotor.get("rB");
        BNO055IMU adaImu = opMode.hardwareMap.get(BNO055IMU.class, "imu");


        DcMotor[] driveMotors = {lF,lB,rF,rB};
        DcMotor[] allMotors = {lF,lB,rF,rB};


        useEncoders(allMotors);
        setBrake(allMotors);

        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);




    }

    void useEncoders(DcMotor[] motorArray){
        for(DcMotor motor : motorArray){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void dontUseEncoders(DcMotor[] motorArray){
        for(DcMotor motor : motorArray){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    void setBrake(DcMotor[] motorArray){
        for(DcMotor motor : motorArray){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void setCoast(DcMotor[] motorArray){
        for(DcMotor motor : motorArray){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

}
