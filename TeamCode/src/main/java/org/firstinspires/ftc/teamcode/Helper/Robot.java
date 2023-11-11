package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU

import java.util.*;

public class Robot {
    /*
    Properties that describe hardware.
     */
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    public Gate gate = new Gate();
    public Chassis chassis = new Chassis();

    /* local OpMode members. */
    //Init hardware map
    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;
        gate.init(hwMap);
        chassis.init(hwMap);
    }
}
