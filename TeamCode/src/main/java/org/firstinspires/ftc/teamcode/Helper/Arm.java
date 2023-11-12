package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm {

    //Object creation
    public DcMotor motor;
    int timeout_ms = 5000;
    double speed = 1;
    int targetPosition;
    int currentPosition;
//    int slowDown;


    public void init(HardwareMap ahwMap) throws InterruptedException {
        HardwareMap hwMap = ahwMap;
        //Init motors and servos
        motor = hwMap.get(DcMotor.class, "Arm");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

// ARM WITH BUTTONS V2
    public void gotoPosition(int targetPosition) {
        ElapsedTime runtime = new ElapsedTime();
        timeout_ms = 3000;
        currentPosition = motor.getCurrentPosition();
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = speed * java.lang.Math.signum(targetPosition - currentPosition);
////        //Set the power of the motor.
        motor.setPower(speed);
        runtime.reset();

        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }
        motor.setPower(0.05); //Holding power.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

}
