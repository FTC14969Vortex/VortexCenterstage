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

        avoidPerimeterPosition = new Vector2d(-36, -62).minus(robotLocalOffset);
        comeBack2_5Pose = new Vector2d(-60, -25).minus(robotLocalOffset);
        comeBack3_4Pose = new Vector2d(-58, -22).minus(robotLocalOffset);
        outtakeCommonPose = new Vector2d(-60, -48).minus(robotLocalOffset);



        //Outake Coordinates for Autos
        outtake16pose = new Vector2d(-38, -45).minus(robotLocalOffset);
        outtake25Pose = new Vector2d(-50, -28).minus(robotLocalOffset);
        outtake34Pose = new Vector2d(-63,-41).minus(robotLocalOffset);

        startBackboardPose = new Vector2d(-36,-12).minus(robotLocalOffset);

        backboardPose = new Vector2d(35, -42).minus(robotLocalOffset);
        parkPose = new Vector2d(50, -72).minus(robotLocalOffset);

    }
}