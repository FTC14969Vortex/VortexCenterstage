package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo servo;

    public double TARGET_POSITION;
    public double CURRENT_POSITION;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        servo = hwMap.get(Servo.class, "Wrist");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(int TARGET_POSITION) {
        CURRENT_POSITION = servo.getPosition();
        servo.setPosition(TARGET_POSITION);

    }
    

}