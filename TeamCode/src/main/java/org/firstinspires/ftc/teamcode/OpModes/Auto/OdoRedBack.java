package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OdoRedBack", group = "Auto")
public class OdoRedBack extends OdometryCommon{

    @Override
    public void setUniqueParameters() {
        IS_AUTO_FRONT = false;
        vision.RED_APRILTAG_OFFSET = 3;

        //Center Tag is 5 for Red, 2 for Blue
        vision.CENTER_TAG_ID = 5;

        //Coordinates for where the robot is initialized
        startPose = new Pose2d(12, -62, Math.toRadians(270));
        avoidPerimeterPosition = new Vector2d(12, -62).minus(robotLocalOffset);
        outtakeCommonPose = new Vector2d(39, -45).minus(robotLocalOffset);

        //Outake Coordinates for Autos
        outtake25Pose = new Vector2d(24, -29.5).minus(robotLocalOffset);
        outtake34Pose = new Vector2d(15.5,-43).minus(robotLocalOffset);

        backboardPose = new Vector2d(45, -42).minus(robotLocalOffset);
        parkPose = new Vector2d(50, -72).minus(robotLocalOffset);

    }
}