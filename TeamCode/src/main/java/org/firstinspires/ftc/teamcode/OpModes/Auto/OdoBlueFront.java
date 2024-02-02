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
        startPose = new Pose2d(-36, 72, Math.toRadians(90));

        avoidPerimeterPosition = new Vector2d(-36, 62);
        outtakeCommonPose = new Vector2d(-60, 48);

        //Outake Coordinates for Autos
        outtake16pose = new Vector2d(-42, 45);
        outtake25Pose = new Vector2d(-48, 33);
        outtake34Pose = new Vector2d(-63,48);

        startBackboardPose = new Vector2d(-36,12);

        backboardPosition = new Vector2d(35, 42);
        parkPosition = new Vector2d(50, 72);

    }
}