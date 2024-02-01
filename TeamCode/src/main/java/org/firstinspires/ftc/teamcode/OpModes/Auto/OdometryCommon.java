package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Helper.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "OdometryCommon", group = "Auto")

public class OdometryCommon extends LinearOpMode{


    /*
    Vision Variables
     */

    //For centering on the AprilTag
    public int TARGET_TAG_ID = 2;            // Start ID as -1, will be updated in the function.
    // X-coordinate of team element at the start of auto.

    public int RED_APRILTAG_OFFSET;

    // Set Unique Parameters

    /*
        Objects of classes
     */
    public Robot robot = new Robot();
    public AutoVision vision = new AutoVision();
    SampleMecanumDrive drive;


    enum AutoStages {DETECT_TE, GOTOOUTTAKE, OUTTAKE, GO_TO_BACKBOARD, DELVER_BACKBOARD, INTAKE_STACK, RETURN_BACKBOARD, PARK, END_AUTO}
    AutoStages currentStage = AutoStages.DETECT_TE;

    /*
        Road Runner Variables; vectors, poses, etc.
     */
    Pose2d startPose;
    Trajectory avoidPerimeter;
    Trajectory goToOutake;
    Trajectory outtake_2_5;
    Trajectory outtake_3_6;
    Trajectory goToBackboard;
    Trajectory park;

    Vector2d avoidPerimeterPosition;
    Vector2d outtakeCommonPosition;
    Vector2d outtake25Position;
    Vector2d outtake36Position;
    Vector2d backboardPosition;
    Vector2d parkPosition;
    Vector2d centerTagPosition;

    public void setUniqueParameters() {
        vision.RED_APRILTAG_OFFSET = 0;
        vision.CENTER_TAG_ID = 5;
        startPose = new Pose2d(12, 72, Math.toRadians(90));
        avoidPerimeterPosition = new Vector2d(12, 62);
        outtakeCommonPosition = new Vector2d(36, 45);
        outtake25Position = new Vector2d(17, 30);
        outtake36Position = new Vector2d(12,45);
        backboardPosition = new Vector2d(45, 48);
        parkPosition = new Vector2d(50, 72);
        centerTagPosition = new Vector2d(60, 65);

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

        RED_APRILTAG_OFFSET = 0;
        vision.CENTER_TAG_ID = 2;


        while (!isStarted()) {
            if (opModeInInit()) {
                vision.detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", TARGET_TAG_ID);
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
                    currentStage = AutoStages.OUTTAKE;
                    break;
                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */

                    switch (vision.TARGET_SPIKE_MARK) {
                        case 1:
                            outtake_1_4();
                            telemetry.addLine("Case 1");
                            break;
                        case 2:
                            outtake_2_5();
                            telemetry.addLine("Case 2");
                            break;
                        case 3:
                            outtake_3_6();
                            telemetry.addLine("Case 3");
                            break;
                    }
                    currentStage = AutoStages.GO_TO_BACKBOARD;
                    break;
                case GO_TO_BACKBOARD:
                    goToBackboard();
                    currentStage = AutoStages.DELVER_BACKBOARD;
                    break;
                case DELVER_BACKBOARD:
                    deliver();
                    currentStage = AutoStages.PARK;
                    break;
                case PARK:
                    park();
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

        avoidPerimeter = drive.trajectoryBuilder(startPose)
                .lineTo(avoidPerimeterPosition)
                .build();

        goToOutake = drive.trajectoryBuilder(avoidPerimeter.end())
                .splineTo(outtakeCommonPosition, Math.toRadians(180))
                .build();


        //Move away so we don't hit the perimeter.
        drive.followTrajectory(avoidPerimeter);

        //Drive to spikemark
        drive.followTrajectory(goToOutake);

    }
    public void outtake_1_4() {
        //Outtake at spike mark
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.6, true);
        sleep(2000);
        robot.intake.MoveIntake(0, false);
    }
    public void outtake_2_5() {
        outtake_2_5 = drive.trajectoryBuilder(goToOutake.end())
                .lineTo(outtake25Position)
                .build();

        drive.followTrajectory(outtake_2_5);

        //Outtake at spike mark
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        sleep(2000);
        robot.intake.MoveIntake(0, false);

        //Come Back
        Trajectory comeBack = drive.trajectoryBuilder(outtake_2_5.end())
                .lineTo(outtakeCommonPosition)
                .build();
        drive.followTrajectory(comeBack);

    }
    public void outtake_3_6() {
        outtake_3_6 = drive.trajectoryBuilder(goToOutake.end())
                .lineTo(outtake36Position)
                .build();

        drive.followTrajectory(outtake_3_6);

        //Outtake at spike mark
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        sleep(2000);
        robot.intake.MoveIntake(0, false);

        //Come back
        Trajectory comeBack = drive.trajectoryBuilder(outtake_3_6.end())
                .lineTo(outtakeCommonPosition)
                .build();
        drive.followTrajectory(comeBack);
    }

    public void goToBackboard() {
        goToBackboard = drive.trajectoryBuilder(goToOutake.end())
                .lineTo(backboardPosition)
                .build();
        drive.followTrajectory(goToBackboard);
    }
    public void deliver() {
        robot.arm.gotoAutoPosition();
        sleep(2000);
        robot.wrist.gotoAutoPosition();
        sleep(1500);
        robot.gate.open();
        sleep(2000);
        robot.wrist.gotoPickupPosition();
        sleep(1000);
        robot.arm.gotoPickupPosition();
        sleep(1000);
    }
    public void park() {
        park = drive.trajectoryBuilder(goToBackboard.end())
                .splineTo(parkPosition, Math.toRadians(180))
                .build();
        drive.followTrajectory(park);

    }

    // initializing Vision

}



