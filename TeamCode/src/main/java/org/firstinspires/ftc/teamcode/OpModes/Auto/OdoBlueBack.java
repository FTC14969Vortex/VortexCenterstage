package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OdoBlueBack", group = "Auto")
public class OdoBlueBack extends OdometryCommon{

    @Override
    public void setUniqueParameters() {
        IS_AUTO_FRONT = false;
        vision.RED_APRILTAG_OFFSET = 0;

        //Center Tag is 5 for Red, 2 for Blue
        vision.CENTER_TAG_ID = 2;

        //Coordinates for where the robot is initialized
        startPose = new Pose2d(12, 62, Math.toRadians(90));

        //Outake Coordinates for Autos
        outtake16Pose = new Vector2d(37, 40).plus(robotLocalOffsetVector);
        outtake25Pose = new Vector2d(12, 20).plus(robotLocalOffsetVector);
        outtake34Pose = new Pose2d(12,45,Math.toRadians(180)).plus(robotLocalOffsetPose);

        comeBack16 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack25 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack34 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);

        backboardPose = new Vector2d(45, 46).plus(robotLocalOffsetVector);

        backUpDistance = 4;

        //edge park
//        parkPose = new Vector2d(50, 72).plus(robotLocalOffset);

        //center park
        parkPose = new Vector2d(53, 27).plus(robotLocalOffsetVector);

    }
}