// ARM WITH BUTTONS V1:
// public class Arm {
//    HardwareMap hwMap = null;
//    public DcMotor motor;
//    int timeout_ms = 5000;
//    double speed;
//    int Position;
//
//
//    public void init(HardwareMap ahwMap) throws InterruptedException {
//
//        hwMap = ahwMap;
//        //Init motors and servos
//        motor = hwMap.get(DcMotor.class, "Arm");
//        motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void swingUp() {
//        ElapsedTime runtime = new ElapsedTime();
//        speed = 0.3;
//        Position = -75;
//        timeout_ms = 500;
//
//        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setTargetPosition(Position);
//        //set the mode to go to the target position
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set the power of the motor.
//        motor.setPower(speed);
//
//        runtime.reset();
//
//        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
//        }
//        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setPower(0);
//
//    }
//
//    public void swingDown() {
//        ElapsedTime runtime = new ElapsedTime();
//        runtime.reset();
//        speed = 0.2;
//        Position = -300;
//
//        motor.setTargetPosition(Position);
//        //set the mode to go to the target position
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set the power of the motor.
//        motor.setPower(speed);
//
//
//        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
//        }
//
//    }
//}


//CODE FOR JOYSTICK CONTROL V1:
package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm {

    //Object creation
    public DcMotor motor;
    int timeout_ms = 5000;
    double speed;
    int Position;
//    int slowDown;


    public void init(HardwareMap ahwMap) throws InterruptedException {
        HardwareMap hwMap = ahwMap;
        //Init motors and servos
        motor = hwMap.get(DcMotor.class, "Arm");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

// ARM WITH BUTTONS V2
//    public void swingUpB() {
//        ElapsedTime runtime = new ElapsedTime();
//        speed = 0.3;
//        Position = 500;
//        timeout_ms = 3000;
//        motor.setTargetPosition(Position);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////        //Set the power of the motor.
//        motor.setPower(speed);
//        runtime.reset();
//
//        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
//        }
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setPower(0);
//
//    }
//
//    public void swingDownB() {
//        ElapsedTime runtime = new ElapsedTime();
//        speed = 0.3;
//        Position = 1000;
//        timeout_ms = 3000;
//        motor.setTargetPosition(Position);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set the power of the motor.
//        motor.setPower(speed);
//
//        runtime.reset();
//
//        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
//        }
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setPower(0);
//    }
}


        // ARM WITH JOYSTICK CODE V2:
//        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void swingUp() {
//        ElapsedTime runtime = new ElapsedTime();
//        speed = 0.3;
//        Position = 235;
//        timeout_ms = 3000;
//        motor.setTargetPosition(Position);
//
////        motor.setTargetPosition(Position);
////        //set the mode to go to the target position
////        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        //Set the power of the motor.
////        motor.setPower(speed);
////
////        runtime.reset();
////
////        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
////        }
////        motor.setPower(0);
//
//    }
//
//    public void swingDown() {
//        ElapsedTime runtime = new ElapsedTime();
//        speed = 0.3;
//        Position = 235;
//        timeout_ms = 3000;
//        motor.setTargetPosition(Position);
//
////        motor.setTargetPosition(Position);
////        //set the mode to go to the target position
////        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        //Set the power of the motor.
////        motor.setPower(speed);
////
////        runtime.reset();
////
////        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
////        }
////        motor.setPower(0);
// }

//
