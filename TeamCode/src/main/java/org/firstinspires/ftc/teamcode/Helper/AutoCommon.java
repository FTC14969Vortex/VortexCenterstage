package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import org.firstinspires.ftc.teamcode.OpModes.AutoBlueFront;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

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
    public void outakeStraight() {
        robot.chassis.Drive(0.7,-24);
        robot.chassis.autoTurn(180);
        robot.chassis.Drive(0.7, 12);
        robot.intake.MoveIntake(0.7,true);
        robot.chassis.Drive(0.7, 12);
    }

    public void outakeRight(float endAngle) {
        org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
        angle = robot.chassis.imu.getAngularOrientation();
        robot.chassis.Drive(0.7, -24);
        robot.chassis.autoTurn(270);
        robot.intake.MoveIntake(0.7,false);
        robot.chassis.Strafe(0.7, 24);
    }
    public void outakeLeft(float endAngle) {
        org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
        angle = robot.chassis.imu.getAngularOrientation();
        robot.chassis.Drive(0.7, -24);
        robot.chassis.autoTurn(90);
        robot.intake.MoveIntake(0.7,false);
        robot.chassis.Strafe(0.7, -24);
    }

    public void goToBackboard() {
        robot.chassis.DriveToPosition(0.7, 96, 24, false);
        robot.chassis.autoTurn(90);
        robot.arm.gotoPosition(ARM_DELIVERY_POSITION);
        robot.wrist.servoPosition(WRIST_DELIVERY_POSITION);
    }

    public void runAuto(String tagValue) {
        switch (tagValue) {
            case "TagTwo":
                outakeStraight();
                goToBackboard();
                break;

        }

    }
}
