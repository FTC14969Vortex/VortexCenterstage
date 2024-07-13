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
        outtake16Pose = new Vector2d(36, -41).minus(robotLocalOffsetVector);
        outtake25Pose = new Vector2d(12, -20.5).minus(robotLocalOffsetVector);
        outtake34Pose = new Pose2d(10,-41, Math.toRadians(180)).minus(robotLocalOffsetPose);

        comeBackPose = new Pose2d(43, -50, Math.toRadians(180)).minus(robotLocalOffsetPose);

        backboardPose = new Vector2d(45, -50).minus(robotLocalOffsetVector);

        backUpDistance = 4;

        cyclePoint = new Vector2d(0,-3);
        stacksPose = new Vector2d(-35,-14);

        //edge park
        parkPose = new Vector2d(65, -80).minus(robotLocalOffsetVector);

        //center park
//        parkPose = new Vector2d(50, -14).minus(robotLocalOffset);

    }
}