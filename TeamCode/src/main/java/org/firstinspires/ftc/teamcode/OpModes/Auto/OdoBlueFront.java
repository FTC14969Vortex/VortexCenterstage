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

        avoidPerimeterPosition = new Vector2d(-36, 62).plus(robotLocalOffset);
        comeBack2_5Pose = new Vector2d(-60, 25).plus(robotLocalOffset);
        comeBack3_4Pose = new Vector2d(-58, 22).plus(robotLocalOffset);
        outtakeCommonPose = new Vector2d(-60, 48).plus(robotLocalOffset);



        //Outake Coordinates for Autos
        outtake16pose = new Vector2d(-38, 43).plus(robotLocalOffset);
        outtake25Pose = new Vector2d(-50, 31).plus(robotLocalOffset);
        outtake34Pose = new Vector2d(-63,43).plus(robotLocalOffset);

        startBackboardPose = new Vector2d(-36,12).plus(robotLocalOffset);

        backboardPose = new Vector2d(35, 42).plus(robotLocalOffset);
        parkPose = new Vector2d(50, 72).plus(robotLocalOffset);

    }
}