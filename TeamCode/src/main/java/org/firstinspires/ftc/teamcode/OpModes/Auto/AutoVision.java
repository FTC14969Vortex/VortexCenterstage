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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
@Autonomous(name = "AutoVision", group = "Auto")
@Disabled
public class AutoVision extends LinearOpMode {
    //------------------------------------------------------------------
    /**
     * Vision Variables.
     */
    public boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    AprilTagDetection centerTag = null;

    // The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal myVisionPortal;

    // Custom model with blue and red team elements.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/FixedLightingCenterstage.tflite";
    private static final String[] LABELS = {
            "Blue",
            "Red"
    };

    double DELIVERY_DISTANCE = 21; //  This is how close the camera should get to the target (inches)
    //For centering on the AprilTag
    public int TARGET_TAG_ID = -1;            // Start ID as -1, will be updated in the function.
    // X-coordinate of team element at the start of auto.
    public float XPOS_OF_TEAM_ELEMENT = 320; // Frame size is 640, default in the middle.
    public int TARGET_SPIKE_MARK = 3;
    public float RECOGNITION_SIZE;

    //------------------------------------------------------------------

    //Robot Object
    public Robot robot = new Robot();

    // Robot control parameters
    double DRIVE_SPEED = 0.7;

    //------------------------------------------------------------------

    /**
     * Variables to change for different autos.
     * The logic in all four OpModes is identical, we only change these variables.
     */

    public int RED_APRILTAG_OFFSET; // 0 for Blue side and 3 for Red side.

    //ID of the center AprilTag
    public int CENTER_TAG_ID = 2;    //2 for blue, 5 for red

    /**
     * Initialize AprilTag and TFOD.
     */
    public void initDoubleVision(HardwareMap hwMap) {
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
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();


        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();

            //Competition exposure
            //setManualExposure(60, 250);  // Use low exposure time to reduce motion blur

            //Garage Exposure

            setManualExposure(200, 250);

        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (myVisionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
            while (!isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
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

    public void detectTeamElement() { // detect position of team element.
        /**
         * Step 1: Detect object
         * (common to all Auto)
         */
        // Enable TFOD to detect object and store the value.
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        myVisionPortal.setProcessorEnabled(tfod, true);


        //Get a recognition
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        float teamElemThresholdMax = 200;
        float teamElemThresholdMin = 25;
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getWidth() > teamElemThresholdMax ||
                    recognition.getWidth() < teamElemThresholdMin) {
                continue;
            }
            XPOS_OF_TEAM_ELEMENT = (recognition.getLeft() + recognition.getRight()) / 2;
            RECOGNITION_SIZE = recognition.getWidth();

            break;
        }


        if (XPOS_OF_TEAM_ELEMENT < 120) {
            TARGET_SPIKE_MARK = 1; //Left
        } else if (XPOS_OF_TEAM_ELEMENT > 440) {
            TARGET_SPIKE_MARK = 3; //Right
        } else {
            TARGET_SPIKE_MARK = 2; //Center
        }


        // Add offset to account for blue or red side.
        TARGET_TAG_ID = TARGET_SPIKE_MARK + RED_APRILTAG_OFFSET;
        // Push telemetry to the Driver Station.


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
                if (detection.id == IDtoDetect) {
                    foundDetection = true;
                    detectionResult = detection;
                    break;
                }
            }
            if (foundDetection) {
                break;
            }

            sleep(300);
        }
        return detectionResult;

    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}