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
        outtake25Pose = new Vector2d(-40, -10).minus(robotLocalOffsetVector);
        outtake34Pose = new Pose2d(-47,-18, Math.toRadians(270)).minus(robotLocalOffsetPose);

        comeBackPose = new Pose2d(-50, -8, Math.toRadians(180)).minus(robotLocalOffsetPose);

        startBackboardPose = new Vector2d(-40,-10).minus(robotLocalOffsetVector);
        cyclePoint = new Vector2d(40,-10).minus(robotLocalOffsetVector);

        backUpDistance = 4;

        backboardPose = new Vector2d(42, -48).minus(robotLocalOffsetVector);

        parkPose = new Vector2d(50, -72).minus(robotLocalOffsetVector);



    }
}