package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Servo {
    public CRServo servo;
    HardwareMap hwMap = null;

   // public double Retract = 0;
    //public double Extend = 1;
    public double INPOWER = 1;
    public double OUTPOWER = -1;

    public void init(HardwareMap ahwMap) throws InterruptedException {
        //servo.setPower(POWER);
        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(CRServo.class, "Intake_Servo");
        servo.setDirection(CRServo.Direction.FORWARD);
    }
    public void Intake() {servo.setPower(INPOWER);
    }

    public void Outake() {servo.setPower(OUTPOWER); }

    public void Stop() {servo.setPower(0);}
}
