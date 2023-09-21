package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class IntakeHelper {

    public DcMotor motor;
    private ElapsedTime runtime = new ElapsedTime();
    int timeout_ms;

    //Init hardware map
    HardwareMap hwMap = null;

    //Initializing hardware maps and motors for intake.
    public void init (HardwareMap ahwMap){
        hwMap = ahwMap;
        motor = hwMap.get(DcMotor.class, "intake");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveIntake (double speed, boolean direction, int timeout) {

//        timeout_ms = timeout;
//        runtime.reset();

        if(direction) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        } else if (!direction) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the power of the motor.
        motor.setPower(speed);

//        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
//        }

//        motor.setPower(0);

    }
}
