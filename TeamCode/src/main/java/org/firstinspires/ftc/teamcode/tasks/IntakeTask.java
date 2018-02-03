package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.DriveTrain;

public class IntakeTask extends TaskThread {

    private DcMotor lIntake, rIntake;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public IntakeTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void run() {
        timer.reset();
        while (opMode.opModeIsActive() && running) {
          /*  if (opMode.gamepad1.dpad_up || opMode.gamepad2.b) {
                setRollerPower(1);
            } else if (opMode.gamepad1.dpad_down || opMode.gamepad2.x) {
                setRollerPower(-1);
            } else{
                setRollerPower(0);
            }*/
            tankControl();


        }
    }
//

    public void setRollerPower(double power) {
        rIntake.setPower(power);
        lIntake.setPower(power);
    }
    public void tankControl(){
        lIntake.setPower(opMode.gamepad2.left_stick_y);
        rIntake.setPower(opMode.gamepad2.right_stick_y);

    }
    @Override
    public void initialize() {
        lIntake = opMode.hardwareMap.dcMotor.get("lIntake");
        rIntake = opMode.hardwareMap.dcMotor.get("rIntake");
        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
