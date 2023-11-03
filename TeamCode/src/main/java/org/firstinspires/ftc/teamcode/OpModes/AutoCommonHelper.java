package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Helper.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.*;

public class AutoCommonHelper {

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


    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //Variables for AprilTags

    double DESIRED_DISTANCE = 6; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.

    public int TagToDetect;

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
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;




    //TensorFlow Detection
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/DetectTeamElement.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "Blue Team Element",
            "Red Team Element",
    };




    /**
     * This is the actual switch case statement that runs the auto
     */
    public void runAuto(String tagValue) throws InterruptedException {
        switch (tagValue) {
            case "Two":
//                outakeStraight();
                goToBackboardAndAdjust();
                deliver();
                break;

        }
    }


    /**
     * Auto Methods
     */

    //Auto Functions
    public void outakeStraight() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED,24);
        robot.intake.MoveIntake(0.6,true);
        robot.chassis.Drive(DRIVE_SPEED,29);
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
    public void goToBackboardAndAdjust() throws InterruptedException {
        robot.chassis.DriveToPosition(DRIVE_SPEED, 80, 26, false);
        robot.chassis.autoTurn(-100);
        driveToAprilTag(TagToDetect);
    }
    public void deliver() throws InterruptedException {
        robot.arm.gotoPosition(ARM_DELIVERY_POSITION);
        robot.wrist.servoPosition(WRIST_DELIVERY_POSITION);
        robot.chassis.Drive(0.3, 2);
        Thread.sleep(1000);
        robot.gate.open();
        Thread.sleep(600);
        robot.gate.close();
        robot.arm.gotoPosition(ARM_PICKUP_POSITION);
    }



    public void driveToAprilTag(int tagID) throws InterruptedException {

        DESIRED_TAG_ID = tagID;

        //Variables for adjusting according to the AprilTags
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0.5;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0.5;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0.5;        // Desired turning power/speed (-1 to +1)


        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null)
                    && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        }

        robot.chassis.moveRobotAprilTags(drive, strafe, turn);
        Thread.sleep(10);

    }
}
