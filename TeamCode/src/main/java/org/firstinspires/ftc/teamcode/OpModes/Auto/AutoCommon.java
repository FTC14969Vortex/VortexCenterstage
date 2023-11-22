/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.Helper.Robot;


import java.util.List;

/*
 * This OpMode illustrates the basics of using both AprilTag recognition and TensorFlow
 * Object Detection.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "AutoCommon", group = "Auto")
public class AutoCommon extends LinearOpMode {
    /**
     * Common variables for all Auto.
     */
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

   // The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;

    AprilTagDetection tag;
    int tagID;
    //Variables for AprilTag delivery

    double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    // The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

   //The variable to store our instance of the vision portal.
    private VisionPortal myVisionPortal;

    // Custom model with blue and red team elements.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/FixedLightingCenterstage.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue",
            "Red"
    };

    // X-coordinate of team element at the start of auto.
    float teamElementX = 320; // Frame size is 640, default in the middle.
    int targetSpikeMark = 2;
    int targetAprilTag = 0; // This needs to be assigned to 1, 2, or 3, based on the detected value.

    float recognition_size;


    //Robot Object
    public Robot robot = new Robot();

    // Robot control parameters
    //Arm and Wrist positions
    int ARM_DELIVERY_POSITION = -600;
    int ARM_PICKUP_POSITION = 4;

    double WRIST_DELIVERY_POSITION = 0.9;
    double WRIST_PICKUP_POSITION = 0.25;

    double DRIVE_SPEED = 0.5;

    enum AutoStages {DETECT_TE, OUTTAKE, GOTO_BACKBOARD, DETECT_AprilTag, CENTER_AprilTag, END_AUTO}
    AutoStages currentStage = AutoStages.DETECT_AprilTag;

    /**
     * Variables to change for different autos.
     * The logic in all four OpModes is identical, we only change these variables.
     */
    // Target value of April tag for yellow pixel.
    public int targetAprilTagOffset; // 0 for Blue side and 3 for Red side.
    // Strafe left or right after delivering purple pixel.
    public int strafeDirAfterPurPix; // Blue autos require strafing right, and red require strafing left.
    public int turnAngleNearBackstage; // Blue side requires counter clockwise turn, red requires CW turn.
    //Distance in inches from the middle of the spike mark 2 to the backstage.
    public int strafeDistAfterPurPix; //24 inches when starting from the back and 24+72 inches when starting from front.


    public void setUniqueParameters(){
        /**
         * Set parameters specific to starting position in Auto here.
         */
        targetAprilTagOffset = 0;
        strafeDirAfterPurPix = -1; // +1 is right for Blue, -1 is left for Red.
        turnAngleNearBackstage = 95;
        strafeDistAfterPurPix = 96;
    }

    int strafeDistAtBackboard;

    /**
     * All methods for the AutoOpMode, we keep them public so that other opmodes can use them.
     */


    //For centering on the AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    boolean centerTagFound = false;     //Set to true when the camera detects the middle april tag
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize
        initDoubleVision();
        setUniqueParameters();
        robot.init(hardwareMap);
//
        detectTeamElement();

        while (!isStarted()) {
            if(opModeInInit()) {
                telemetryAprilTag();
                telemetryTfod();
                telemetry.update();
            }
        }


        waitForStart();


        boolean doneAuto = false;
        while (opModeIsActive() && currentStage != AutoStages.END_AUTO)  {

            switch(currentStage){
                case DETECT_TE:
                    detectTeamElement();
                    telemetryTfod();
                    telemetry.update();
                    currentStage = AutoStages.OUTTAKE;
                    break;
                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */

                    switch(targetSpikeMark){
                        case 1:
                            outTakeLeft();
                            strafeDistAtBackboard = 38;

                            break;
                        case 2:
                            outTakeStraight();
                            strafeDistAtBackboard = 36;
                            break;
                        case 3:
                            outTakeRight();
                            strafeDistAtBackboard = 44;
                            break;
                    }
                    currentStage = AutoStages.GOTO_BACKBOARD;
                    break;
                case GOTO_BACKBOARD:
                    /**
                     * Step 3: Drive to Backstage.
                     *
                     */
                    gotoBackBoard(strafeDirAfterPurPix,strafeDistAfterPurPix,turnAngleNearBackstage, strafeDistAtBackboard);
                    currentStage = AutoStages.DETECT_AprilTag;
                    break;
                case DETECT_AprilTag:
                    detect_apriltag();
                    telemetryAprilTag();
                    telemetry.update();
                    currentStage = AutoStages.CENTER_AprilTag;
                    break;
                case CENTER_AprilTag:
                    strafetoMiddleTag();
                    currentStage = AutoStages.END_AUTO;
                    break;
                case END_AUTO:

            }






            doneAuto = true;


        } // end while loop

    }  //end opMode



    /**
     * Initialize AprilTag and TFOD.
     */
    public void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1612.13,1612.13, 265.34, -257.457)
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();


        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(myVisionPortal.getProcessorEnabled(aprilTag)) {
            telemetry.addLine("April Tags working");
        }

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("yaw",detection.ftcPose.yaw);
                telemetry.addData("range",detection.ftcPose.range);
                telemetry.addData("current detection", tagID);
                telemetry.update();
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public double telemetryTfod() {

        double x = 0;
        double y;

        if(myVisionPortal.getProcessorEnabled(tfod)) {
            telemetry.addLine("TFOD Tags working");
        }

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        return x;

    }   // end method telemetryTfod()

    public void detectTeamElement(){ // detect position of team element.
        /**
         * Step 1: Detect object
         * (common to all Auto)
         */
        // Enable TFOD to detect object and store the value.
        myVisionPortal.setProcessorEnabled(tfod, true);
        myVisionPortal.setProcessorEnabled(aprilTag, true);


        //Get a recognition
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        float teamElemThresholdMax = 200;
        float teamElemThresholdMin = 25;
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getWidth() > teamElemThresholdMax ||
                    recognition.getWidth() < teamElemThresholdMin) {
                continue;
            }
            teamElementX = (recognition.getLeft() + recognition.getRight()) / 2;
            recognition_size = recognition.getWidth();

            break;
        }


        if(teamElementX < 120) {
            targetSpikeMark = 3; //Left
        } else if(teamElementX > 440) {
            targetSpikeMark = 1; //Right
        } else {
            targetSpikeMark = 2; //Center
        }



        // Add offset to account for blue or red side.
        targetAprilTag = targetSpikeMark + targetAprilTagOffset;

        // Push telemetry to the Driver Station.
        telemetry.update();


    }

    /**
     * Methods for driving the robot.
     */
    public void outTakeStraight() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED,51);
        robot.intake.MoveIntake(0.3,true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0,true);
    }

    public void outTakeLeft() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED, 27);
        robot.chassis.autoTurn(-100);
        robot.intake.MoveIntake(0.3,true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0,true);
        robot.chassis.Drive(DRIVE_SPEED, 2);
        robot.chassis.Strafe(DRIVE_SPEED,35);
        robot.chassis.autoTurn(95);

    }
    public void outTakeRight() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED, 27);
        robot.chassis.autoTurn(90);
        robot.intake.MoveIntake(0.3,true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, true);
        robot.chassis.Drive(DRIVE_SPEED, 2);
        robot.chassis.Strafe(DRIVE_SPEED, -35);
        robot.chassis.autoTurn(-95);
    }

    public void gotoBackBoard(int strafeDirAfterPurPix, int strafeDistAfterPurPix, int turnAngleNearBackstage, int strafeDistAtBackboard) throws InterruptedException {
        robot.chassis.Strafe(DRIVE_SPEED, strafeDirAfterPurPix*strafeDistAfterPurPix);
        robot.chassis.autoTurn(turnAngleNearBackstage);
        robot.chassis.Strafe(DRIVE_SPEED, strafeDirAfterPurPix*strafeDistAtBackboard);
    }



    public void moveRobotAprilTags(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.chassis.FLMotor.setPower(leftFrontPower);
        robot.chassis.FRMotor.setPower(rightFrontPower);
        robot.chassis.BLMotor.setPower(leftBackPower);
        robot.chassis.BRMotor.setPower(rightBackPower);
    }
    public void centerOnAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == targetAprilTag) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    tag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if(targetFound) {
            double  rangeError      = (tag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = tag.ftcPose.bearing;
            double  yawError        = tag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        }
        moveRobotAprilTags(drive, strafe, turn);
    }
    public void moveYaw(double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  -yaw;
        double rightFrontPower   =  +yaw;
        double leftBackPower     =  -yaw;
        double rightBackPower    =  +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.chassis.FLMotor.setPower(leftFrontPower);
        robot.chassis.FRMotor.setPower(rightFrontPower);
        robot.chassis.BLMotor.setPower(leftBackPower);
        robot.chassis.BRMotor.setPower(rightBackPower);
    }
    public void correctYaw(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if the middle tag is detected
                if (detection.id == 2) {
                    // Yes, we want to use this middle tag to adjust the yaw
                    centerTagFound = true;
                    tag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        if(centerTagFound) {
            double  yawError        = tag.ftcPose.yaw;
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        }
        moveYaw(turn);
    }
    public void strafetoMiddleTag(){
        if(tag!=null){
            robot.chassis.Strafe(0.5, (int)tag.ftcPose.x);
        }
        if(centerTagFound) {
            double  yawError        = tag.ftcPose.yaw;
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        }
        moveYaw(turn);
    }
    public void detect_apriltag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.id == 2) {
                // Yes, we want to use this middle tag to adjust the yaw
                tag = detection;
                tagID = detection.id;
                break;  // don't look any further.}
            }
        }
    }
}   // end class