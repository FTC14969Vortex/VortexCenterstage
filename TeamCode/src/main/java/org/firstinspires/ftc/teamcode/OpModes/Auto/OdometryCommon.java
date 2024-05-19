package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Helper.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "OdometryCommon", group = "Auto")

public class OdometryCommon extends LinearOpMode{

    /*
    Vision Variables
     */

    //For centering on the AprilTag
    // X-coordinate of team element at the start of auto.

    public int RED_APRILTAG_OFFSET;

    // Set Unique Parameters

    /*
        Objects of classes
     */
    public Robot robot = new Robot();
    public AutoVision vision = new AutoVision();
    SampleMecanumDrive drive;


    enum AutoStages {DETECT_TE, OUTTAKE, DELIVER_BACKBOARD, INTAKE_STACK, RETURN_BACKBOARD, PARK, END_AUTO}
    AutoStages currentStage = AutoStages.DETECT_TE;

    /*
        Road Runner Variables; vectors, poses, etc.
     */
    boolean IS_AUTO_FRONT;

    //TRAJECTORIES
    Trajectory outtake_1_6;
    Trajectory outtake_2_5;
    Trajectory outtake_3_4;
    Trajectory comeBack;
    Trajectory startBackboard;
    Trajectory extendToBackboard;
    Trajectory FrontSideBackboard;
    Trajectory BackSideBackboard;
    Trajectory backintoBoard;
    Trajectory park;
    Trajectory moveToDeliveryTag;
    Trajectory driveStacks;
    Trajectory runStacks;

    int backUpDistance;

    //VECTORS AND POSITIONS
    Pose2d startPose;
    Vector2d outtake16Pose;
    Vector2d outtake25Pose;
    Pose2d outtake34Pose;
    Pose2d comeBackPose;
    Vector2d startBackboardPose;
    Vector2d cyclePoint;
    Vector2d backboardPose;
    Vector2d stacksPose;
    Vector2d parkPose;
    Pose2d robotLocalOffsetPose = new Pose2d(0,-6.5,Math.toRadians(0));
    Vector2d robotLocalOffsetVector = new Vector2d(0,-6.5);

    public void setUniqueParameters() {
        IS_AUTO_FRONT = false;
        vision.RED_APRILTAG_OFFSET = 0;

        //Center Tag is 5 for Red, 2 for Blue
        vision.CENTER_TAG_ID = 5;

        //Coordinates for where the robot is initialized
        startPose = new Pose2d(12, 72, Math.toRadians(90));
        outtake16Pose = new Vector2d(37, 45);
        outtake25Pose = new Vector2d(20, 36);
        outtake34Pose = new Pose2d(12,45,Math.toRadians(270));
        comeBackPose = new Pose2d(-50, -7, 180).plus(robotLocalOffsetPose);
        startBackboardPose = new Vector2d(-12,12);
        cyclePoint = new Vector2d(0,0);
        backboardPose = new Vector2d(45, 48);
        backUpDistance = 5;
        stacksPose = new Vector2d(20, 36);
        parkPose = new Vector2d(48, 72);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        setUniqueParameters();

        robot.intake.init(hardwareMap);
        robot.arm.init(hardwareMap);
        robot.wrist.init(hardwareMap);
        robot.gate.init(hardwareMap);

        //Finish creating drive object
        drive = new SampleMecanumDrive(hardwareMap);

        vision.initDoubleVision(hardwareMap);


        while (!isStarted()) {
            if (opModeInInit()) {
                vision.detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                telemetry.update();
                robot.gate.close();
            }
        }

        waitForStart();


        while (opModeIsActive()) {
            switch (currentStage) {
                case DETECT_TE:
                    vision.detectTeamElement();
                    telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                    telemetry.update();
                    currentStage = AutoStages.OUTTAKE;
                    break;

                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */
                    switch (vision.TARGET_SPIKE_MARK) {
                        case 1:
                            outtake_1_6();
                            telemetry.addLine("Case 1");
                            break;
                        case 2:
                            outtake_2_5();
                            telemetry.addLine("Case 2");
                            break;
                        case 3:
                            outtake_3_4();
                            telemetry.addLine("Case 3");
                            break;
                    }
//                    sleep(2000);
                    currentStage = AutoStages.DELIVER_BACKBOARD;
                    break;


                case DELIVER_BACKBOARD:
                    gotoBackBoard();
                    centerOnTarget();
                    delivery();
                    currentStage = AutoStages.INTAKE_STACK;
                    break;

                case PARK:
                    park();
                    currentStage = AutoStages.END_AUTO;
                    break;
                case INTAKE_STACK:
                    goToStacks();
                    currentStage = AutoStages.END_AUTO;
                case END_AUTO:
                    // End Auto keeps printing debug information via telemetry.
                    telemetry.update();
                    sleep(5000); //5 sec delay between telemetry.
            }
        }
    }

    public  void outtake(){
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        sleep(2000);
        robot.intake.MoveIntake(0, false);
    }
    public void outtake_1_6() throws InterruptedException{
        if (!IS_AUTO_FRONT) {
            drive.setPoseEstimate(startPose);
            outtake_1_6 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(
                            new Pose2d(outtake16Pose.getX(),outtake16Pose.getY(),Math.toRadians(180))
                    )
                    .build();

            comeBack = drive.trajectoryBuilder(outtake_1_6.end())
                    .lineToLinearHeading(comeBackPose)
                    .build();

            drive.followTrajectory(outtake_1_6);
            //Outtake at spike mark
            outtake();

            drive.followTrajectory(comeBack);
        } else {
            drive.setPoseEstimate(startPose);
            outtake_1_6 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(
                            new Pose2d(outtake16Pose.getX(),outtake16Pose.getY(),Math.toRadians(0))
                    )
                    .build();
            comeBack = drive.trajectoryBuilder(outtake_1_6.end())
                    .lineToLinearHeading(comeBackPose)
                    .build();

            drive.followTrajectory(outtake_1_6);
            outtake();
            drive.followTrajectory(comeBack);
        }
    }
    public void outtake_2_5() {
        drive.setPoseEstimate(startPose);
        if (!IS_AUTO_FRONT){
            outtake_2_5 = drive.trajectoryBuilder(startPose)
                    .lineTo(outtake25Pose)
                    .build();

            comeBack = drive.trajectoryBuilder(outtake_2_5.end())
                    .lineToLinearHeading(new Pose2d(outtake16Pose.getX(),outtake16Pose.getY(),Math.toRadians(180)))
                    .build();

            drive.followTrajectory(outtake_2_5);

            //Outtake at spike mark
            outtake();
            //Return to Outtake common position
            drive.followTrajectory(comeBack);

        } else {
            outtake_2_5 = drive.trajectoryBuilder(startPose)
                    .lineTo(outtake25Pose)
                    .build();
            comeBack = drive.trajectoryBuilder(outtake_2_5.end())
                    .lineToLinearHeading(new Pose2d(comeBackPose.getX(),comeBackPose.getY(),Math.toRadians(180)))
                    .build();

            drive.followTrajectory(outtake_2_5);

            //Outtake at spike mark
            outtake();
            //Return to Outtake common position
            drive.followTrajectory(comeBack);
        }


    }
    public void outtake_3_4() {
        drive.setPoseEstimate(startPose);
        if (!IS_AUTO_FRONT){
            outtake_3_4 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(outtake34Pose)
                    .build();

            comeBack = drive.trajectoryBuilder(outtake_3_4.end())
                    .lineToLinearHeading(comeBackPose)
                    .build();

            drive.followTrajectory(outtake_3_4);

            //Outtake at spike mark
            outtake();

            //Return to Outtake common position
            drive.followTrajectory(comeBack);
        } else {
            outtake_3_4 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(outtake34Pose)
                    .build();

            comeBack = drive.trajectoryBuilder(outtake_3_4.end())
                    .lineToLinearHeading(comeBackPose)
                    .build();

            drive.followTrajectory(outtake_3_4);

            //Outtake at spike mark
            outtake();

            //Return to Outtake common position
            drive.followTrajectory(comeBack);
        }
    }
    public void gotoBackBoard() {

        if (!IS_AUTO_FRONT) {
            BackSideBackboard = drive.trajectoryBuilder(comeBack.end())
                    .lineTo(backboardPose)
                    .build();
            drive.followTrajectory(BackSideBackboard);
        } else {
            startBackboard = drive.trajectoryBuilder(comeBack.end())
                    .lineTo(startBackboardPose)
                    .build();
            extendToBackboard = drive.trajectoryBuilder(startBackboard.end())
                    .lineTo(cyclePoint)
                    .build();
            FrontSideBackboard = drive.trajectoryBuilder(extendToBackboard.end())
                    .lineToLinearHeading(new Pose2d(backboardPose.getX(), backboardPose.getY(), Math.toRadians(180)))
                    .build();
            drive.followTrajectory(startBackboard);
            drive.followTrajectory(extendToBackboard);
            drive.followTrajectory(FrontSideBackboard);
        }
    }
    public double getRangeError() {

        AprilTagDetection tempTag = null;
        double rangeError = 0;

        tempTag = vision.detect_apriltag(vision.CENTER_TAG_ID);
        if (tempTag != null) {
            rangeError = (tempTag.ftcPose.range / 2.54) - vision.DELIVERY_DISTANCE;
            telemetry.addData("range error", rangeError);

        }
        return rangeError;
    }

    public void centerOnTarget() {
        telemetry.addData("CentertagID", vision.CENTER_TAG_ID);
        telemetry.addData("TargetTagID", vision.TARGET_TAG_ID);
        telemetry.update();
        double rangeError = getRangeError();
        if(!IS_AUTO_FRONT){
            double adjustedRangeX = BackSideBackboard.end().getX() + rangeError;
            if(vision.TARGET_TAG_ID < vision.CENTER_TAG_ID) {
                moveToDeliveryTag = drive.trajectoryBuilder(BackSideBackboard.end())
                        .lineTo(new Vector2d(adjustedRangeX , (BackSideBackboard.end().getY() + 5.5)))
                        .build();

                drive.followTrajectory(moveToDeliveryTag);
            }
            else if(vision.TARGET_TAG_ID > vision.CENTER_TAG_ID) {
                moveToDeliveryTag = drive.trajectoryBuilder(BackSideBackboard.end())
                        .lineTo(new Vector2d(adjustedRangeX, (BackSideBackboard.end().getY() - 5.5)))
                        .build();

                drive.followTrajectory(moveToDeliveryTag);
            }
            else{
                if (rangeError != 0){
                    moveToDeliveryTag = drive.trajectoryBuilder(BackSideBackboard.end())
                            .lineTo(new Vector2d(adjustedRangeX, (BackSideBackboard.end().getY())))
                            .build();

                    drive.followTrajectory(moveToDeliveryTag);
                }
            }
            backintoBoard = drive.trajectoryBuilder(moveToDeliveryTag.end())
                    .lineTo(
                            new Vector2d(moveToDeliveryTag.end().getX()+ 1.5, moveToDeliveryTag.end().getY()),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
        } else {
            double adjustedRangeX = FrontSideBackboard.end().getX() + rangeError;
            if(vision.TARGET_TAG_ID < vision.CENTER_TAG_ID) {
                moveToDeliveryTag = drive.trajectoryBuilder(FrontSideBackboard.end())
                        .lineTo(new Vector2d(adjustedRangeX , (FrontSideBackboard.end().getY() + 4.75)))
                        .build();

                drive.followTrajectory(moveToDeliveryTag);
            }
            else if(vision.TARGET_TAG_ID > vision.CENTER_TAG_ID) {
                moveToDeliveryTag = drive.trajectoryBuilder(FrontSideBackboard.end())
                        .lineTo(new Vector2d(adjustedRangeX, (FrontSideBackboard.end().getY() - 10)))
                        .build();

                drive.followTrajectory(moveToDeliveryTag);
            }
            else if (rangeError != 0){
                moveToDeliveryTag = drive.trajectoryBuilder(FrontSideBackboard.end())
                        .lineTo(new Vector2d(adjustedRangeX, (FrontSideBackboard.end().getY())))
                        .build();

                drive.followTrajectory(moveToDeliveryTag);

            } else {
                //Failsafe if Camera doesn't detect anything
                moveToDeliveryTag = drive.trajectoryBuilder(FrontSideBackboard.end())
                        .lineTo(new Vector2d(FrontSideBackboard.end().getX()-0.1, (FrontSideBackboard.end().getY()-0.1)))
                        .build();

                drive.followTrajectory(moveToDeliveryTag);


            }
            backintoBoard = drive.trajectoryBuilder(moveToDeliveryTag.end())
                    .lineTo(
                            new Vector2d(moveToDeliveryTag.end().getX()+ backUpDistance, moveToDeliveryTag.end().getY()),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
        }


    }

    public void goToStacks() {
        if (!IS_AUTO_FRONT) {
            driveStacks = drive.trajectoryBuilder(backintoBoard.end())
                    .lineTo(cyclePoint)
                    .build();
            drive.followTrajectory(driveStacks);
        }

    }
    public void park() {
        if (!IS_AUTO_FRONT) {
            park = drive.trajectoryBuilder(backintoBoard.end())
                    .lineTo(parkPose)
                    .build();
            drive.followTrajectory(park);
        } else {
            park = drive.trajectoryBuilder(backintoBoard.end())
                    .lineTo(parkPose)
                    .build();
            drive.followTrajectory(park);
        }

    }

    public void delivery() {

        robot.gate.close();
        robot.arm.gotoAutoPosition();
        telemetry.addData("Encoder Val", robot.arm.motor.getCurrentPosition());
        telemetry.update();
        sleep(150);
        robot.wrist.gotoAutoPosition();

        sleep(1500);
        drive.followTrajectory(backintoBoard);
        sleep(1100);

        robot.gate.open();
        sleep(550);
        robot.wrist.gotoPickupPosition();
        sleep(2000);
        robot.arm.gotoPickupPosition();
        sleep(500);
    }

}