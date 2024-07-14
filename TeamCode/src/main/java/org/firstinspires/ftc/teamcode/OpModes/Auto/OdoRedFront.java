package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OdoRedFront", group = "Auto")
public class OdoRedFront extends OdometryCommon{
    @Override
    public void setUniqueParameters() {
        IS_AUTO_FRONT = true;
        vision.RED_APRILTAG_OFFSET = 3;

        //Center Tag is 5 for Red, 2 for Blue
        vision.CENTER_TAG_ID = 5;

        //Coordinates for where the robot is initialized
        startPose = new Pose2d(-36, -62, Math.toRadians(270));


        //Outake Coordinates for Autos
        outtake16Pose = new Vector2d(-38, -23 ).minus(robotLocalOffsetVector);
        outtake25Pose = new Vector2d(-36, -19).minus(robotLocalOffsetVector);
        outtake25Pose = new Vector2d(-12, -12).minus(robotLocalOffsetVector);
        outtake34Pose = new Pose2d(-47,-18, Math.toRadians(270)).minus(robotLocalOffsetPose);
        outtake342Pose = new Pose2d(-9,-21, Math.toRadians(180)).minus(robotLocalOffsetPose);

        comeBack16 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack25 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack34 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);

        startBackboardPose = new Vector2d(-40,-10).minus(robotLocalOffsetVector);
        cyclePoint = new Vector2d(40,-10).minus(robotLocalOffsetVector);

        backUpDistance = 3;

        backboardPose16 = new Vector2d(45, -50).minus(robotLocalOffsetVector);
        backboardPose25 = new Vector2d(45, -50).minus(robotLocalOffsetVector);
        backboardPose34 = new Vector2d(40, -45).minus(robotLocalOffsetVector);

        parkFirstPose = new Vector2d(65, -80).minus(robotLocalOffsetVector);
        parkSecondPose = new Vector2d(40, -80).minus(robotLocalOffsetVector);


    }
}