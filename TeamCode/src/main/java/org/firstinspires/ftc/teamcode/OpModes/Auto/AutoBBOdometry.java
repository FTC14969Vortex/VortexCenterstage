package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous(name = "AutoBBOdometry", group = "Auto")

/*
class TrajPoses{
    public Vector2d start;
    public Vector2d end;

    public TrajPoses(Vector2d _start, Vector2d _end){
        start = _start;
        end = _end;
*/

public class AutoBBOdometry extends LinearOpMode {
    public Robot robot = new Robot();
    public AutoVision vision = new AutoVision();
    // Initialize
    double DRIVE_SPEED = 0.8;

    // setUniqueParameters();
    enum AutoStages {DETECT_TE,GOTOOUTTAKE, OUTTAKE, GOTO_BACKBOARD, CENTER_AprilTag, DELVER_BACKBOARD_PARK, END_AUTO}
    AutoBBOdometry.AutoStages currentStage = AutoBBOdometry.AutoStages.DETECT_TE;
    // What direction to strafe to move to the backboard
    public int STRAFE_TO_BACKBOARD_DIRECTION; // Blue autos require strafing right, and red require strafing left.

    //What distance to strafe to move to the backboard
    public int STRAFE_TO_BACKBOARD_DISTANCE; //24 inches when starting from the back and 24+72 inches when starting from front.

    //What angle to turn for the camera to face the backboard
    public int TURN_ANGLE_TO_FACE_BACKBOARD; // Blue side requires counter clockwise turn, red requires clockwise turn.

    //What distance to strafe so the camera can see the middle tag
    public int STRAFE_TO_MIDDLE_TAG_DISTANCE;

    //What direction to strafe to park.
    public int STRAFE_DIRECTION_FOR_PARKING;

    public int STRAFE_DISTANCE;
    public void setUniqueParameters() {
        vision.RED_APRILTAG_OFFSET = 0;
        STRAFE_TO_BACKBOARD_DIRECTION = -1; // +1 is right for Blue, -1 is left for Red.
        TURN_ANGLE_TO_FACE_BACKBOARD = 95;
        STRAFE_TO_BACKBOARD_DISTANCE = 96;
        STRAFE_TO_MIDDLE_TAG_DISTANCE = -39;
        STRAFE_DIRECTION_FOR_PARKING = -1;
        vision.CENTER_TAG_ID = 5;

    }

    public void runOpMode() throws InterruptedException {
        //Robot Object
        vision.initDoubleVision(hardwareMap);
        while (!isStarted()) {
            if (opModeInInit()) {
                vision.detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                vision.telemetryAprilTag();
                telemetry.update();
            }
        }

        waitForStart();

        while (opModeInInit()) {
            switch (currentStage) {
                case DETECT_TE:
                    vision.detectTeamElement();
                    telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                    telemetry.update();
                    currentStage = AutoStages.GOTOOUTTAKE;
                    break;
                case GOTOOUTTAKE:
                    outakeCommon();
                    currentStage = AutoStages.OUTTAKE;
                    break;
                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */

                    switch (vision.TARGET_SPIKE_MARK) {
                        case 1:
                            odometry1and4();
                            break;
                        case 2:
                            odometry2and5();
                            break;
                        case 3:
                            odometry3and6();
                            break;
                    }
                    currentStage = AutoStages.GOTO_BACKBOARD;
                    break;
                case GOTO_BACKBOARD:
                    odometryToBackboard();
                    currentStage = AutoStages.CENTER_AprilTag;
                    break;
                case CENTER_AprilTag:
//                    vision.TARGET_TAG_ID = 6; //Overriding the target tag for testing.
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
                    telemetry.update();
                    sleep(5000); //5 sec delay between telemetry.
            }

        } // end while loop

    }  //end opMode
    public void deliverToBackboardAndPark() {

        // Swing the arm and wist to low position.
        robot.arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.gotoAutoPosition();
        sleep(100);
        robot.wrist.gotoAutoPosition();
        sleep(1500);

        // Open the gate to deliver one pixel.
        robot.gate.open();
        sleep(1000);

        // Bring the wrist and arm to pickup position.
        robot.wrist.gotoPosition(robot.wrist.WRIST_PICKUP_POSITION);
        sleep(850);
        robot.arm.gotoPickupPosition();

        // Park.
        robot.chassis.Strafe(DRIVE_SPEED, 3 - vision.TARGET_SPIKE_MARK * 6 + (STRAFE_DISTANCE*STRAFE_DIRECTION_FOR_PARKING));
        robot.chassis.Drive(DRIVE_SPEED, 13);
    }
    public void centerToCenterTag() {
        double yawError = 0;
        double yawCorrection = 0;
        double rotation_comp = 7;
        float turnOffsetAprilTag = 0;
        double edgeOffset = 7.5;
        AprilTagDetection tempTag = null;


        tempTag = vision.detect_apriltag(vision.CENTER_TAG_ID);
        if (tempTag != null) {
            vision.centerTag = tempTag; //Update the center tag if detection was successful.
        }
//        tempTag = vision.detect_apriltag(vision.CENTER_TAG_ID);
//        if (tempTag != null) {
//            vision.centerTag = tempTag; //Update the center tag if detection was successful.
//        }
        if (vision.centerTag != null) {
            yawError = vision.centerTag.ftcPose.yaw;
            if (yawError > -3){
                yawCorrection += rotation_comp;
            }
            if (Math.abs(yawCorrection) > 3) {
                robot.chassis.autoTurn((float) -yawCorrection, turnOffsetAprilTag);
                sleep(2000);

            }
        }


        // Use the speed and turn "gains" to calculate how we want the robot to move.
        // turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


        tempTag = vision.detect_apriltag(vision.CENTER_TAG_ID);
        if (tempTag != null) {
            vision.centerTag = tempTag;
        }
        if (vision.centerTag != null) {
            double strafeError = vision.centerTag.ftcPose.x;
            if (vision.TARGET_TAG_ID < vision.CENTER_TAG_ID) {
                strafeError -= edgeOffset;
            } else if (vision.TARGET_TAG_ID > vision.CENTER_TAG_ID) {
                strafeError += edgeOffset;
            }

            robot.chassis.Strafe(DRIVE_SPEED, strafeError); //x is in inches.
            sleep(500);
            telemetry.addData("strafe error", strafeError);
        }


        tempTag = vision.detect_apriltag(vision.TARGET_TAG_ID);
        if (tempTag != null) {
            double rangeError = (tempTag.ftcPose.range / 2.54) - vision.DELIVERY_DISTANCE;
            robot.chassis.Drive(DRIVE_SPEED * 0.5, (float) rangeError);
            telemetry.addData("range error", rangeError);

        }



        sleep(500);
        telemetry.addData("yaw error", yawError);
        telemetry.addData("yaw correction", yawCorrection);

        //telemetry.addData("measured range", tempTag.ftcPose.range);



    }
    public void outakeCommon() throws InterruptedException {
        // Initialize
        // robot.init(hardwareMap);
        vision.initDoubleVision(hardwareMap);
        // setUniqueParameters();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.intake.init(hardwareMap);
        robot.arm.init(hardwareMap);
        robot.wrist.init(hardwareMap);
        robot.gate.init(hardwareMap);



        Pose2d startPose = new Pose2d(12, 72, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        //TrajPoses outakePart1 = new TrajPoses(new Vector2d(12, 62), new Vector2d(14, 72));


//
        Trajectory outakePart1 = drive.trajectoryBuilder(startPose)
//                .back(48)
                .lineTo(new Vector2d(12, 62))
                .build();

        Trajectory outakePart2 = drive.trajectoryBuilder(outakePart1.end())
                .splineTo(new Vector2d(36, 45), Math.toRadians(180))
                //.splineTo(new Vector2d(9, -10), 0)
                .build();
//
        Trajectory goToBackboard = drive.trajectoryBuilder(outakePart2.end())
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0)
                .build();

        waitForStart();

        outakePart1.velocity(50);
        drive.followTrajectory(outakePart1);
        //drive.turn(Math.toRadians(90));
//        robot.intake.MoveIntake(0, false);
        drive.followTrajectory(outakePart2);
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);
        drive.followTrajectory(goToBackboard);
    }


    public void odometry1and4() throws InterruptedException {
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);
    }

    public void odometry2and5() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.intake.init(hardwareMap);
        Trajectory twoandfivePart1 = drive.trajectoryBuilder(new Pose2d(36,45),Math.toRadians(180))
                .lineTo(new Vector2d(33,42))
                .build();
        drive.followTrajectory(twoandfivePart1);
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);
        Trajectory twoandfivePart2 = drive.trajectoryBuilder(twoandfivePart1.end(),Math.toRadians(180))
                .lineTo(new Vector2d(36,45))
                .build();
        drive.followTrajectory(twoandfivePart2);
    }
    public void odometry3and6() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.intake.init(hardwareMap);
        Trajectory threeandsixPart1 = drive.trajectoryBuilder(new Pose2d(36,45),Math.toRadians(180))
                .lineTo(new Vector2d(30,45))
                .build();
        drive.followTrajectory(threeandsixPart1);
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);
        Trajectory threeandsixPart2 = drive.trajectoryBuilder(threeandsixPart1.end(),Math.toRadians(180))
                .lineTo(new Vector2d(36,45))
                .build();
        drive.followTrajectory(threeandsixPart2);
    }
    public void odometryToBackboard() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.intake.init(hardwareMap);
        Trajectory goToBackboard = drive.trajectoryBuilder(new Pose2d(36,45),Math.toRadians(180))
                .lineTo(new Vector2d(39,45))
                .build();
        drive.followTrajectory(goToBackboard);
    }

}


