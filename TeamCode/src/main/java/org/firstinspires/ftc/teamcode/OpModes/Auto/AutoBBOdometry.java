package org.firstinspires.ftc.teamcode.OpModes.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpModes.Auto.AutoCommon;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

@Autonomous(name = "AutoBBOdometry", group = "Auto")

public class AutoBBOdometry extends AutoCommon {

    public void outakeCommon() throws InterruptedException {
        // Initialize
        // robot.init(hardwareMap);
        // vision.initDoubleVision(hardwareMap);
        setUniqueParameters();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.intake.init(hardwareMap);
        robot.arm.init(hardwareMap);
        robot.wrist.init(hardwareMap);
        robot.gate.init(hardwareMap);




        Pose2d startPose = new Pose2d(12, 72, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
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

    //    @Override
    public void RunOdometry() throws InterruptedException {

        // Initialize
        robot.init(hardwareMap);
        vision.initDoubleVision(hardwareMap);
        setUniqueParameters();


        while (!isStarted()) {
            if (opModeInInit()) {
                vision.detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                vision.telemetryAprilTag();
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


