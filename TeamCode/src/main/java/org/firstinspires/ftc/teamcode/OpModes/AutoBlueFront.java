package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Blue Front", group = "Auto")
public class AutoBlueFront extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    Robot robot = new Robot();

    public enum AutoSteps {
        detectSpikeMarks, deliverPurple, deliverYellow, endAuto
    }

    public AutoSteps Step = AutoSteps.detectSpikeMarks;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        initDoubleVision();


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        robot.myVisionPortal.setProcessorEnabled(robot.aprilTag, true);
        robot.myVisionPortal.setProcessorEnabled(robot.tfod, true);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (robot.myVisionPortal.getProcessorEnabled(robot.aprilTag)) {
                    telemetry.addLine("AprilTag Detection Working");
                    telemetry.addLine();
                    telemetryAprilTag();
                }
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
                if (robot.myVisionPortal.getProcessorEnabled(robot.tfod)) {
                    telemetry.addLine("TFOD Detection Working");
                    telemetry.addLine();
                    telemetryTfod();
                }

                telemetry.update();

                switch (Step) {
                    case detectSpikeMarks:
                        telemetry.addLine("Detected Team Element");
                        telemetry.update();
                        Step = AutoSteps.deliverPurple;
                        break;

                    case deliverPurple:
                        telemetry.addLine("Delivered purple pixel on spike marks!");
                        telemetry.update();
                        Step = AutoSteps.deliverYellow;

                        // Deliver preload and park.
                    case deliverYellow:
                        telemetry.addLine("Delivered yellow pixel on backboard");
                        telemetry.update();
                        Step = AutoSteps.endAuto;
                        break;

                    case endAuto:
                        telemetry.addLine("Auto Finished!");
                        telemetry.update();
                        break;
                }
            }
        }
    }




    /**
     * Function to initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        robot.aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        robot.tfod = new TfodProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (robot.USE_WEBCAM) {
            robot.myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(robot.tfod, robot.aprilTag)
                    .build();
        } else {
            robot.myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(robot.tfod, robot.aprilTag)
                    .build();
        }
    }   // end initDoubleVision()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag () {
        List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()


    /**
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod () {
        List<Recognition> currentRecognitions = robot.tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}