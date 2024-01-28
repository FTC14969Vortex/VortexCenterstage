package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gate {
    public Servo servo;

    public double OPEN = 1;
    public double CLOSE = 0.65;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(Servo.class, "Gate");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void open() {
        servo.setPosition(OPEN);
    }

    public void close() {
        servo.setPosition(CLOSE);
    }


}








