package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Helper.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "OdometryCommon", group = "Auto")

public class OdometryCommon extends LinearOpMode{


    /*
    Vision Variables
     */

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

    //For centering on the AprilTag
    public int TARGET_TAG_ID = 2;            // Start ID as -1, will be updated in the function.
    // X-coordinate of team element at the start of auto.
    public float XPOS_OF_TEAM_ELEMENT = 320; // Frame size is 640, default in the middle.
    public int TARGET_SPIKE_MARK = 2;

    public int RED_APRILTAG_OFFSET;
    public float RECOGNITION_SIZE;

    // Set Unique Parameters

    /*
        Objects of classes
     */
    public Robot robot = new Robot();
    public AutoVision vision = new AutoVision();
    SampleMecanumDrive drive;
    enum AutoStages {DETECT_TE, GOTOOUTTAKE, OUTTAKE, GOTO_BACKBOARD, DELVER_BACKBOARD_PARK, END_AUTO}
    AutoStages currentStage = AutoStages.DETECT_TE;

    /*
        Road Runner Variables; vectors, poses, etc.
     */
    Pose2d startPose = new Pose2d(12, 72, Math.toRadians(90));
    Vector2d outakePart1end = new Vector2d(12, 62);
    Vector2d outakePart2end = new Vector2d(36, 45);
    Vector2d goToBackboardend = new Vector2d(45, 48);
    Vector2d parkend = new Vector2d(50, 72);
    Vector2d strafeToCenterTag = new Vector2d(60, 65);
    Vector2d rangeCorrect = new Vector2d(51, 48);


    public void setUniqueParameters() {
        vision.RED_APRILTAG_OFFSET = 0;
        vision.CENTER_TAG_ID = 5;

    }
    
    @Override
    public void runOpMode() throws InterruptedException {

        robot.intake.init(hardwareMap);
        robot.arm.init(hardwareMap);
        robot.wrist.init(hardwareMap);
        robot.gate.init(hardwareMap);

        //Finish creating drive object
        drive = new SampleMecanumDrive(hardwareMap);

        vision.initDoubleVision(hardwareMap);

        RED_APRILTAG_OFFSET = 0;
        vision.CENTER_TAG_ID = 2;



        while (!isStarted()) {
            if (opModeInInit()) {
                vision.detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", TARGET_TAG_ID);
//                vision.telemetryAprilTag();
                telemetry.update();
            }
        }

        waitForStart();


        while (opModeIsActive()) {
            switch (currentStage) {
                case DETECT_TE:
                    vision.detectTeamElement();
                    telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                    telemetry.update();
                    currentStage = AutoStages.GOTOOUTTAKE;
                    break;
                case GOTOOUTTAKE:
                    outakeCommon();
                    currentStage = AutoStages.GOTOOUTTAKE;
                    break;
                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */

                    switch (vision.TARGET_SPIKE_MARK) {
                        case 1:
                            telemetry.addLine("Case 1");
                            break;
                        case 2:
                            telemetry.addLine("Case 2");
                            break;
                        case 3:
                            telemetry.addLine("Case 3");
                            break;
                    }
                    currentStage = AutoStages.GOTO_BACKBOARD;
                    break;
                case GOTO_BACKBOARD:
                    currentStage = AutoStages.DELVER_BACKBOARD_PARK;
                    break;
                case DELVER_BACKBOARD_PARK:
                    currentStage = AutoStages.END_AUTO;
                    break;
                case END_AUTO:
                    // End Auto keeps printing debug information via telemetry.
                    telemetry.update();
                    sleep(5000); //5 sec delay between telemetry.
            }
        }
    }

    public void outakeCommon() throws InterruptedException {

        drive.setPoseEstimate(startPose);

        Trajectory backOut = drive.trajectoryBuilder(startPose)
                .lineTo(outakePart1end)
                .build();

        Trajectory outakePart2 = drive.trajectoryBuilder(backOut.end())
                .splineTo(outakePart2end, Math.toRadians(180))
                .build();

        Trajectory goToBackboard = drive.trajectoryBuilder(outakePart2.end())
                .lineTo(goToBackboardend)
                .build();

        Trajectory park = drive.trajectoryBuilder(goToBackboard.end())
                .splineTo(parkend, Math.toRadians(180))
                .build();


        //Move away so we don't hit the perimeter.
        drive.followTrajectory(backOut);

        //Stop intake
        robot.intake.MoveIntake(0, false);

        //Drive to spikemark
        drive.followTrajectory(outakePart2);

        //Outtake at spike mark
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);

        //Go to backboard
        drive.followTrajectory(goToBackboard);

        //Delivery
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

        //Park
        drive.followTrajectory(park);
    }

    // initializing Vision

}



