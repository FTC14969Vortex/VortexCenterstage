// AutoBlueFront.java
package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "AutoBlueBack", group = "Auto")
public class AutoBlueBack extends AutoCommon {

    @Override
    public void setUniqueParameters() {

        //Add an offset of 3 for the red tags
        vision.RED_APRILTAG_OFFSET = 0; //0 for blue, 3 for red

        // What direction to strafe to move to the backboard
        STRAFE_TO_BACKBOARD_DIRECTION = -1; // -1 for blue, 1 for red

        //What distance to strafe to move to the backboard
        STRAFE_TO_BACKBOARD_DISTANCE = 24; //24 inches when starting from the back and 96 inches when starting from front.

        //What angle to turn for the camera to face the backboard
        TURN_ANGLE_TO_FACE_BACKBOARD = -100; // Blue side requires counter clockwise turn, red requires clockwise turn.

        //What distance to strafe so the camera can see the middle tag
        STRAFE_TO_MIDDLE_TAG_DISTANCE = -37; //constant for both sides

        //What direction to strafe to park.
        STRAFE_DIRECTION_FOR_PARKING = -1; //-1 for blue, 1 for red

        //How far the robot needs to move to be out of the way of the backboard.
        STRAFE_DISTANCE = 20; // 20 for blue, 25 for red.

        //Use the new path that goes under the truss, or the original path that goes under the gate
        USE_NEW_PATH = false; //True for new path, false for old path

        //Center tag for each side
        vision.CENTER_TAG_ID = 2; //2 for blue, 5 for red
    }

}