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
        startPose = new Pose2d(12, -72, Math.toRadians(270));
        avoidPerimeterPosition = new Vector2d(12, -62);
        outtakeCommonPose = new Vector2d(36, -45);

        //Outake Coordinates for Autos
        outtake25Pose = new Vector2d(20, -33);
        outtake36Pose = new Vector2d(12,-45);

        backboardPosition = new Vector2d(45, -48);
        parkPosition = new Vector2d(50, -72);

    }
}