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
        avoidPerimeterPosition = new Vector2d(12, 62).plus(robotLocalOffset);
        outtakeCommonPose = new Vector2d(39, 45).plus(robotLocalOffset);

        //Outake Coordinates for Autos
        outtake16pose = new Vector2d(-38, 45).plus(robotLocalOffset);
        outtake25Pose = new Vector2d(24, 32.5).plus(robotLocalOffset);
        outtake34Pose = new Vector2d(16,47).plus(robotLocalOffset);

        backboardPose = new Vector2d(45, 42).plus(robotLocalOffset);
        parkPose = new Vector2d(50, 72).plus(robotLocalOffset);

    }
}