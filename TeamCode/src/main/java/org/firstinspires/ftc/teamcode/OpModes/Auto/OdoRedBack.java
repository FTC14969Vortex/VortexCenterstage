package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
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


        //Outake Coordinates for Autos
        outtake16Pose = new Vector2d(37, -26).minus(robotLocalOffsetVector);
        outtake25Pose = new Vector2d(12, -15).minus(robotLocalOffsetVector);
        outtake252Pose = new Vector2d(38, -12).minus(robotLocalOffsetVector);
        outtake34Pose = new Pose2d(9,-36, Math.toRadians(270)).minus(robotLocalOffsetPose);
        outtake342Pose = new Pose2d(9,-37, Math.toRadians(180)).minus(robotLocalOffsetPose);


        comeBack16 = new Pose2d(43, -40, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack25 = new Pose2d(43, -40, Math.toRadians(180)).minus(robotLocalOffsetPose);
        comeBack34 = new Pose2d(43, -40, Math.toRadians(180)).minus(robotLocalOffsetPose);

        backboardPose16 = new Vector2d(40, -43).minus(robotLocalOffsetVector);
        backboardPose25 = new Vector2d(40, -43).minus(robotLocalOffsetVector);
        backboardPose34 = new Vector2d(40, -43).minus(robotLocalOffsetVector);

        backUpDistance = 4;

        cyclePoint = new Vector2d(0,-3);
        stacksPose = new Vector2d(-35,-14);

        //edge park
        parkFirstPose = new Vector2d(40, -72).minus(robotLocalOffsetVector);
        parkSecondPose = new Vector2d(65, -72).minus(robotLocalOffsetVector);


        //center park
//        parkPose = new Vector2d(50, -14).minus(robotLocalOffset);

    }
}