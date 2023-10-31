package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class AutoCommon {

    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    //Robot Object
    public Robot robot = new Robot();


    public enum TagValues {
        TagOne, TagTwo, TagThree
    }

    public TagValues TagValue;


    //Arm and Wrist positions
    int ARM_DELIVERY_POSITION = -600;
    int ARM_PICKUP_POSITION = 4;

    double WRIST_DELIVERY_POSITION = 0.9;
    double WRIST_PICKUP_POSITION = 0.25;

    double DRIVE_SPEED = 0.5;


    //Vision


    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
     */
    public TfodProcessor tfod;

    /**
     * {@link #myVisionPortal} is the variable to store our instance of the vision portal.
     */
    public VisionPortal myVisionPortal;

    //Auto Functions
    public void outakeStraight() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED,-24);
//        robot.chassis.autoTurn(180);
        robot.chassis.Drive(DRIVE_SPEED, 28);
        robot.intake.MoveIntake(0.6,true);
        Thread.sleep(1000);
        robot.intake.MoveIntake(0,true);

//        robot.chassis.Drive(DRIVE_SPEED, 12);
    }

    public void outakeLeft() {
        robot.chassis.Drive(DRIVE_SPEED, -24);
        robot.chassis.autoTurn(270);
        robot.intake.MoveIntake(DRIVE_SPEED,false);
        robot.chassis.Strafe(DRIVE_SPEED, 24);
    }
    public void outakeRight() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED, -24);
        robot.chassis.autoTurn(90);
        robot.intake.MoveIntake(DRIVE_SPEED,true);
        Thread.sleep(1000);
        robot.chassis.Drive(DRIVE_SPEED, -2);
        robot.chassis.Strafe(DRIVE_SPEED, -24);
        robot.chassis.autoTurn(180);
//        robot.chassis.Strafe(0.5, -24);
    }

    public void goToBackboardAndDeliver() throws InterruptedException {
        robot.chassis.DriveToPosition(DRIVE_SPEED, 96, 26, false);
        robot.chassis.autoTurn(-95);
        robot.arm.gotoPosition(ARM_DELIVERY_POSITION);
        robot.wrist.servoPosition(WRIST_DELIVERY_POSITION);
        robot.chassis.Drive(0.3, 1);
        Thread.sleep(1000);
        robot.gate.open();
    }

    public void runAuto(String tagValue) throws InterruptedException {
        switch (tagValue) {
            case "TagTwo":
                outakeStraight();
                goToBackboardAndDeliver();
                break;

        }

    }
}
