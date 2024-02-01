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
        vision.CENTER_TAG_ID = 5;

        //Coordinates for where the robot is initialized
        startPose = new Pose2d(-36, 72, Math.toRadians(90));

        avoidPerimeterPosition = new Vector2d(-36, 62);
        outtakeBackCommonPose = new Vector2d(36, 45);
        outtakeFrontCommonPose = new Vector2d(-60, 48);

        //Outake Coordinates for Autos
        outtake25Pose = new Vector2d(-48, 36);
        outtake36Pose = new Vector2d(-48,45);

        backboardPosition = new Vector2d(45, 48);
        parkPosition = new Vector2d(50, 72);

    }
}