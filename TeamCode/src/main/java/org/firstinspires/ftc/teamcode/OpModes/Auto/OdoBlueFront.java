package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OdoBlueFront", group = "Auto")
public class OdoBlueFront extends OdometryCommon{
    @Override
    public void setUniqueParameters() {
        IS_AUTO_FRONT = true;
        vision.RED_APRILTAG_OFFSET = 0;

        //Center Tag is 5 for Red, 2 for Blue
        vision.CENTER_TAG_ID = 2;

        //Coordinates for where the robot is initialized
        startPose = new Pose2d(-36, 68, Math.toRadians(90));

        //Outake Coordinates for Autos
        outtake16Pose = new Vector2d(-38, 43).plus(robotLocalOffsetVector);
        outtake25Pose = new Vector2d(-40, 26).plus(robotLocalOffsetVector);
        outtake34Pose = new Pose2d(-48,34,Math.toRadians(90)).plus(robotLocalOffsetPose);

        comeBack16 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack25 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack34 = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);

        startBackboardPose = new Vector2d(-40,20).plus(robotLocalOffsetVector);
        cyclePoint = new Vector2d(40,20);

        backboardPose16 = new Vector2d(45, -50).minus(robotLocalOffsetVector);
        backboardPose25 = new Vector2d(45, -50).minus(robotLocalOffsetVector);
        backboardPose34 = new Vector2d(45, -50).minus(robotLocalOffsetVector);

        backUpDistance = 4;

        //edge park
//        parkPose = new Vector2d(50, 72).plus(robotLocalOffset);

        //center park
        parkFirstPose = new Vector2d(65, -80).minus(robotLocalOffsetVector);
        parkSecondPose = new Vector2d(40, -80).minus(robotLocalOffsetVector);    }
}