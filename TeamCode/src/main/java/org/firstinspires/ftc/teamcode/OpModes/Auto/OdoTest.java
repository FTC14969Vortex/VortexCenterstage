package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Helper.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Disabled
@Autonomous(name = "OdoTest", group = "Auto")



public class OdoTest extends LinearOpMode {



    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // The variable to store our instance of the AprilTag processor.
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;
    public VisionPortal myVisionPortal;

    // Custom model with blue and red team elements.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/FixedLightingCenterstage.tflite";
    private static final String[] LABELS = {
            "Blue",
            "Red"
    };


    Vector2d rangeCorrect = new Vector2d(51,48);

    //For centering on the AprilTag
    public int TARGET_TAG_ID = 2;            // Start ID as -1, will be updated in the function.
    // X-coordinate of team element at the start of auto.
    public float XPOS_OF_TEAM_ELEMENT = 320; // Frame size is 640, default in the middle.
    public int TARGET_SPIKE_MARK = 2;

    public int RED_APRILTAG_OFFSET;
    public float RECOGNITION_SIZE;



    public Robot robot = new Robot();
    public AutoCommon common = new AutoCommon();
    Pose2d startPose = new Pose2d(12, 72, Math.toRadians(90));
    Vector2d outakePart1end = new Vector2d(12,62);
    Vector2d outakePart2end = new Vector2d(36,45);
    Vector2d goToBackboardend = new Vector2d(47,48);
    Vector2d park1 = new Vector2d(45, 70);
    Vector2d strafeToCenterTag = new Vector2d(60, 65);

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.intake.init(hardwareMap);
        robot.arm.init(hardwareMap);
        robot.wrist.init(hardwareMap);
        robot.gate.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        initDoubleVision();

        RED_APRILTAG_OFFSET = 0;
        common.vision.CENTER_TAG_ID = 2;



        while (!isStarted()) {
            if (opModeInInit()) {
                detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", TARGET_TAG_ID);
//                common.vision.telemetryAprilTag();
                telemetry.update();
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        outakeCommon();
    }





    public void outakeCommon() throws InterruptedException {




        drive.setPoseEstimate(startPose);

        Trajectory outakePart1 = drive.trajectoryBuilder(startPose)
                .lineTo(outakePart1end)
                .build();

        Trajectory outakePart2 = drive.trajectoryBuilder(outakePart1.end())
                .splineTo(outakePart2end, Math.toRadians(180))
                .build();

        Trajectory goToBackboard = drive.trajectoryBuilder(outakePart2.end())
                .lineTo(goToBackboardend)
                .build();
        Trajectory centerToTargetTag = drive.trajectoryBuilder(goToBackboard.end())
                .strafeLeft(10)
                .build();
        Trajectory rangeError = drive.trajectoryBuilder(goToBackboard.end())
                .lineTo(rangeCorrect)
                .build();




        drive.followTrajectory(outakePart1);

        robot.intake.MoveIntake(0, false);

        drive.followTrajectory(outakePart2);

        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);

        drive.followTrajectory(goToBackboard);

        centerToCenterTag(drive);

        robot.arm.gotoAutoPosition();
        Thread.sleep(2000);
        robot.wrist.gotoAutoPosition();
        Thread.sleep(1500);
        robot.gate.open();
        Thread.sleep(2000);
        robot.wrist.gotoPickupPosition();
        Thread.sleep(1000);
        robot.arm.gotoPickupPosition();
        Thread.sleep(1000);


        //drive.followTrajectory();
    }

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
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();


        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();

            //Competition exposure
            //setManualExposure(60, 250);  // Use low exposure time to reduce motion blur

            //Garage Exposure
            common.vision.setManualExposure(200, 250);

        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

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


    public void centerToCenterTag(SampleMecanumDrive drive) {
        double yawError;
        double edgeOffset = 7.5;
        AprilTagDetection tempTag = null;


        tempTag = detect_apriltag(common.vision.CENTER_TAG_ID);
        if (tempTag != null) {
            common.vision.centerTag = tempTag; //Update the center tag if detection was successful.
        }

        //Yaw Correction
        if (common.vision.centerTag != null) {
            yawError = common.vision.centerTag.ftcPose.yaw;
            if (Math.abs(yawError) > 6) {
                drive.turn(Math.toRadians(yawError));
            }
            telemetry.addData("Yaw Error", yawError);
            telemetry.update();
            sleep(500);

        }
        if (tempTag != null) {
            double rangeError = (tempTag.ftcPose.range / 2.54) - common.vision.DELIVERY_DISTANCE;
            robot.chassis.Drive(common.DRIVE_SPEED * 0.5, (float) rangeError);
            telemetry.addData("range error", rangeError);
        }



            // Use the speed and turn "gains" to calculate how we want the robot to move.
        // turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


        tempTag = detect_apriltag(common.vision.CENTER_TAG_ID);
        if (tempTag != null) {
            common.vision.centerTag = tempTag;
        }
        if (common.vision.centerTag != null) {
            double strafeError = common.vision.centerTag.ftcPose.x;
            if (common.vision.TARGET_TAG_ID < common.vision.CENTER_TAG_ID) {
                strafeError -= edgeOffset;
            } else if (common.vision.TARGET_TAG_ID > common.vision.CENTER_TAG_ID) {
                strafeError += edgeOffset;
            }


//            if(strafeError < 0) {
//                strafe(common.DRIVE_SPEED, strafeError); //x is in inches.
//            }

//            sleep(500);
//            telemetry.addData("strafe error", strafeError);
        }


//        tempTag = detect_apriltag(TARGET_TAG_ID);
////        if (tempTag != null) {
////            double rangeError = (tempTag.ftcPose.range / 2.54) - common.vision.DELIVERY_DISTANCE;
////            robot.chassis.Drive(common.DRIVE_SPEED * 0.5, (float) rangeError);
////            telemetry.addData("range error", rangeError);
//
//        }
//
//        sleep(500);
    }



}
