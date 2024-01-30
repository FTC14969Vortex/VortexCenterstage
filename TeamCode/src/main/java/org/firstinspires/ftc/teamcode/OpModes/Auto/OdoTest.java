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
@Autonomous(name = "OdoTest", group = "Auto")

/*
class TrajPoses{
    public Vector2d start;
    public Vector2d end;

    public TrajPoses(Vector2d _start, Vector2d _end){
        start = _start;
        end = _end;
*/

public class OdoTest extends LinearOpMode {
    public Robot robot = new Robot();
    public AutoVision vision = new AutoVision();

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
                .lineTo(new Vector2d(48, 48))
                .build();

        Trajectory park = drive.trajectoryBuilder(outakePart2.end())
                .lineTo(new Vector2d(48, 65))
                .lineTo(new Vector2d(65, 65))
                .build();

        waitForStart();

        outakePart1.velocity(50);
        drive.followTrajectory(outakePart1);
        robot.intake.MoveIntake(0, false);
        drive.followTrajectory(outakePart2);
        robot.intake.motor.setPower(0.5);
        robot.intake.MoveIntake(0.5, true);
        Thread.sleep(2000);
        robot.intake.MoveIntake(0, false);
        drive.followTrajectory(goToBackboard);
        robot.arm.gotoHighPosition();
        Thread.sleep(1000);
        robot.wrist.gotoHighPosition();
        Thread.sleep(1000);
        robot.gate.open();
        Thread.sleep(1000);
        robot.wrist.gotoPickupPosition();
        Thread.sleep(1000);
        robot.arm.gotoPickupPosition();
        Thread.sleep(1000);
        drive.followTrajectory(park);



    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;

        outakeCommon();
    }
}
