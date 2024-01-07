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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.Helper.Robot;


import java.util.List;
import java.util.concurrent.TimeUnit;

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

    AprilTagDetection centerTag = null;
    AprilTagDetection targetTag = null;
    double DELIVERY_DISTANCE = 21; //  this is how close the camera should get to the target (inches)
    //For centering on the AprilTag
    int targetTagID = 6;            // Start ID as -1, will be updated in the function.
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    String debugAutoSequence;
    String debugDetectedAprilTags = "";


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
    int targetAprilTag = 1; // This needs to be assigned to 1, 2, or 3, based on the detected value.

    float recognition_size;


    //Robot Object
    public Robot robot = new Robot();

    // Robot control parameters

    double DRIVE_SPEED = 0.7;
    float turnOffset = 10;

    enum AutoStages {DETECT_TE, OUTTAKE, GOTO_BACKBOARD, CENTER_AprilTag, DELVER_BACKBOARD_PARK, END_AUTO}

    AutoStages currentStage = AutoStages.DETECT_TE;

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
    public int strafeDistAtBackboard;
    public int strafeDirForParking;
    public int redOrBlueSide;
    public int centerTagID;             // Middle AprilTag


    public void setUniqueParameters() {
        /**
         * Set parameters specific to starting position in Auto here.
         */
        targetAprilTagOffset = 0;
        strafeDirAfterPurPix = -1; // +1 is right for Blue, -1 is left for Red.
        turnAngleNearBackstage = 95;
        strafeDistAfterPurPix = 96;
        strafeDistAtBackboard = -39;
        redOrBlueSide = 1;
        strafeDirForParking = -1;
        centerTagID = 5;

    }


    /**
     * All methods for the AutoOpMode, we keep them public so that other opmodes can use them.
     */


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize
        initDoubleVision();
        setUniqueParameters();
        robot.init(hardwareMap);

        while (!isStarted()) {
            if (opModeInInit()) {
                detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", targetTagID);
                telemetryAprilTag();
                telemetry.update();
            }
        }


        waitForStart();

        while (opModeIsActive()) {
            debugAutoSequence = debugAutoSequence + " " + currentStage.toString();
            switch (currentStage) {
                case DETECT_TE:
                    detectTeamElement();
                    telemetry.addData("Target Tag ID", targetTagID);
                    telemetry.update();
                    currentStage = AutoStages.OUTTAKE;
                    break;
                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */

                    switch (targetSpikeMark) {
                        case 1:
                            outTake14();
                            //Red Offset
//                            if(redOrBlueSide == 1) {
//                                strafeDistAtBackboard = 30;
//                            }
//                            //Blue
//                            if(redOrBlueSide == -1) {
//                                strafeDistAtBackboard = -38;
//                            }
                            break;
                        case 2:
                            outTake25();
                            //Red Offset
//                            if(redOrBlueSide == 1) {
//                                strafeDistAtBackboard = 36;
//                            }
//                            //Blue Offset
//                            if(redOrBlueSide == 1) {
//                                strafeDistAtBackboard = -36;
//                            }
                            break;
                        case 3:
                            outTake36();
                            //Red Offset
//                            if(redOrBlueSide == 1) {
//                                strafeDistAtBackboard = 38;
//                            }
//                            //Blue
//                            if(redOrBlueSide == -1) {
//                                strafeDistAtBackboard = -30;
//                            }
                            break;
                    }
                    currentStage = AutoStages.GOTO_BACKBOARD;
                    break;
                case GOTO_BACKBOARD:
                    /**
                     * Step 3: Drive to Backstage.
                     *
                     */
                    sleep(4000);
                    gotoBackBoard(strafeDirAfterPurPix, strafeDistAfterPurPix, turnAngleNearBackstage, strafeDistAtBackboard);
                    currentStage = AutoStages.CENTER_AprilTag;
                    break;
                case CENTER_AprilTag:
                    centerToCenterTag();
                    currentStage = AutoStages.DELVER_BACKBOARD_PARK;
                    break;
                case DELVER_BACKBOARD_PARK:
                    deliverToBackboardAndPark();
                    sleep(1000);
                    currentStage = AutoStages.END_AUTO;
                    break;
                case END_AUTO:
                    // End Auto keeps printing debug information via telemetry.
                    telemetry.addData("auto sequence", debugAutoSequence);
//                    telemetry.addData("Target Tag",targetTag.id);
//                    telemetry.addData("Center Tag",centerTag.id);
                    telemetry.addData("apriltag debugging", debugDetectedAprilTags);
                    telemetry.update();
                    sleep(5000); //5 sec delay between telemetry.
            }

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
                .setLensIntrinsics(1612.13, 1612.13, 265.34, -257.457)
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
            setManualExposure(50, 250);  // Use low exposure time to reduce motion blur

        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    /*
   Manually set the camera gain and exposure.
   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
  */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (myVisionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        myVisionPortal.setProcessorEnabled(tfod, false);
        myVisionPortal.setProcessorEnabled(aprilTag, true);
        if (myVisionPortal.getProcessorEnabled(aprilTag)) {
            telemetry.addLine("April Tags working");
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("yaw", detection.ftcPose.yaw);
                telemetry.addData("range", detection.ftcPose.range);
                telemetry.addData("X", detection.ftcPose.x);
                telemetry.addData("Y", detection.ftcPose.y);
                telemetry.addData("Z", detection.ftcPose.z);
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
        myVisionPortal.setProcessorEnabled(tfod, true);
        myVisionPortal.setProcessorEnabled(aprilTag, false);

        double x = 0;
        double y;

        if (myVisionPortal.getProcessorEnabled(tfod)) {
            telemetry.addLine("TFOD Tags working");
        }

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


        return x;

    }   // end method telemetryTfod()

    public void detectTeamElement() { // detect position of team element.
        /**
         * Step 1: Detect object
         * (common to all Auto)
         */
        // Enable TFOD to detect object and store the value.
        myVisionPortal.setProcessorEnabled(tfod, true);
        myVisionPortal.setProcessorEnabled(aprilTag, false);


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


        if (teamElementX < 120) {
            targetSpikeMark = 1; //Left
        } else if (teamElementX > 440) {
            targetSpikeMark = 3; //Right
        } else {
            targetSpikeMark = 2; //Center
        }


        // Add offset to account for blue or red side.
        targetTagID = targetSpikeMark + targetAprilTagOffset;
        // Push telemetry to the Driver Station.


    }

    /**
     * Methods for driving the robot.
     */
    public void outTake25() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED, 36);
        robot.chassis.autoTurn(180, turnOffset);
        robot.intake.MoveIntake(0.4, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, true);
        robot.chassis.Drive(DRIVE_SPEED, -30);
    }

    public void outTake36() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED, 27);
        robot.chassis.autoTurn(-93, turnOffset);
        robot.chassis.Drive(DRIVE_SPEED, 3);
        robot.intake.MoveIntake(0.4, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, true);
        robot.chassis.Drive(DRIVE_SPEED, -2);
        robot.chassis.autoTurn(93, turnOffset);
        robot.chassis.Drive(DRIVE_SPEED, -30);


    }

    public void outTake14() throws InterruptedException {
        robot.chassis.Drive(DRIVE_SPEED, 27);
        robot.chassis.autoTurn(93, turnOffset);
        robot.chassis.Drive(DRIVE_SPEED, 3);
        robot.intake.MoveIntake(0.4, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, true);
        robot.chassis.Drive(DRIVE_SPEED, -3);
        robot.chassis.autoTurn(-90, turnOffset);
        robot.chassis.Drive(DRIVE_SPEED, -30);
    }

    public void gotoBackBoard(int strafeDirAfterPurPix, int strafeDistAfterPurPix, int turnAngleNearBackstage, int strafeDistAtBackboard) throws InterruptedException {
        robot.chassis.Strafe(DRIVE_SPEED, strafeDirAfterPurPix * strafeDistAfterPurPix);
        robot.chassis.autoTurn(turnAngleNearBackstage, turnOffset);
        robot.chassis.Strafe(DRIVE_SPEED, strafeDistAtBackboard);
    }

    public void centerToCenterTag() {
        double yawError = 0;
        float turnOffsetAprilTag = 0;
        double edgeOffset = 7;
        AprilTagDetection tempTag = null;
        centerTag = detect_apriltag(centerTagID);
        debugDetectedAprilTags = debugDetectedAprilTags + "centerTagID:" + centerTagID + "targetTagID:" + targetTagID;


        if (centerTag != null) {
            yawError = centerTag.ftcPose.yaw;
            if (yawError != 0) {
                robot.chassis.autoTurn((float) -yawError, turnOffsetAprilTag);
                sleep(500);
            }

            debugDetectedAprilTags = debugDetectedAprilTags + "\n yaw correction:" + -yawError;
        }

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        // turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


        tempTag = detect_apriltag(centerTagID);
        if (tempTag != null) {
            centerTag = tempTag; //Update the center tag if detection was successful.
        }

        double strafeError = centerTag.ftcPose.x;
        if (targetTagID < centerTagID) {
            strafeError -= edgeOffset;
        } else if (targetTagID > centerTagID) {
            strafeError += edgeOffset;
        }

        robot.chassis.Strafe(DRIVE_SPEED, strafeError); //x is in inches.
        sleep(500);

        debugDetectedAprilTags = debugDetectedAprilTags + "\n strafe correction:" + strafeError;


        tempTag = detect_apriltag(targetTagID);
        if (tempTag != null) {
            centerTag = tempTag;
        }
        double rangeError = centerTag.ftcPose.y / 2.54 - DELIVERY_DISTANCE;

        robot.chassis.Drive(DRIVE_SPEED * 0.5, (float) rangeError);

        debugDetectedAprilTags = debugDetectedAprilTags + "\n range correction:" + rangeError;


        sleep(500);

    }


    public AprilTagDetection detect_apriltag(int IDtoDetect) {
        myVisionPortal.setProcessorEnabled(tfod, false);
        myVisionPortal.setProcessorEnabled(aprilTag, true);
        boolean foundDetection = false;
        int detect_count = 0;
        AprilTagDetection detectionResult = null;
        List<AprilTagDetection> currentDetections = null;
        for (int i = 0; i < 5; i++) {
            currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on the tag.
                debugDetectedAprilTags = debugDetectedAprilTags + detection.id + detection.ftcPose.yaw + detection.ftcPose.x + ';';

                if (detection.id == IDtoDetect) {
                    foundDetection = true;
                    detectionResult = detection;
                    break;
                }
            }
            if (foundDetection) {
                break;
            }

            sleep(500);
        }
        return detectionResult;

    }


    public void deliverToBackboardAndPark() {

        // Swing the arm and wist to low position.
        robot.arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.gotoAutoPosition();
        robot.wrist.gotoAutoPosition();
        sleep(1000);

        // Open the gate to deliver one pixel.
        robot.gate.open();
        sleep(2000);

        // Bring the wrist and arm to pickup position.
        robot.wrist.gotoPickupPosition();
        sleep(850);
        robot.arm.gotoPickupPosoition();

        // Park.
        robot.chassis.Strafe(DRIVE_SPEED, 3 - targetSpikeMark * 6 + 24);
        robot.chassis.Drive(DRIVE_SPEED, 9);
    }



}