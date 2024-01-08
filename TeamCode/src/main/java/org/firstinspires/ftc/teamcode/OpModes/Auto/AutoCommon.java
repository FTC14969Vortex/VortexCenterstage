/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Helper.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous(name = "AutoCommon", group = "Auto")
public class AutoCommon extends LinearOpMode {
    //------------------------------------------------------------------

    //Robot Object
    public Robot robot = new Robot();
    public AutoVision vision = new AutoVision();


    // Robot control parameters
    double DRIVE_SPEED = 0.7;
    float TURN_OFFSET = 10;

    enum AutoStages {DETECT_TE, OUTTAKE, GOTO_BACKBOARD, CENTER_AprilTag, DELVER_BACKBOARD_PARK, END_AUTO}

    AutoStages currentStage = AutoStages.DETECT_TE;

    //------------------------------------------------------------------

    /**
     * Variables to change for different autos.
     * The logic in all four OpModes is identical, we only change these variables.
     */


    // What direction to strafe to move to the backboard
    public int STRAFE_TO_BACKBOARD_DIRECTION; // Blue autos require strafing right, and red require strafing left.

    //What distance to strafe to move to the backboard
    public int STRAFE_TO_BACKBOARD_DISTANCE; //24 inches when starting from the back and 24+72 inches when starting from front.

    //What angle to turn for the camera to face the backboard
    public int TURN_ANGLE_TO_FACE_BACKBOARD; // Blue side requires counter clockwise turn, red requires clockwise turn.

    //What distance to strafe so the camera can see the middle tag
    public int STRAFE_TO_MIDDLE_TAG_DISTANCE;

    //What direction to strafe to park.
    public int STRAFE_DIRECTION_FOR_PARKING;

    //Use the new path that goes under the truss, or the original path that goes under the gate
    public boolean USE_NEW_PATH; //True for new path, false for old path


    //------------------------------------------------------------------

    /**
     * Set parameters specific to starting position in Auto here.
     */

    public void setUniqueParameters() {
        vision.RED_APRILTAG_OFFSET = 0;
        STRAFE_TO_BACKBOARD_DIRECTION = -1; // +1 is right for Blue, -1 is left for Red.
        TURN_ANGLE_TO_FACE_BACKBOARD = 95;
        STRAFE_TO_BACKBOARD_DISTANCE = 96;
        STRAFE_TO_MIDDLE_TAG_DISTANCE = -39;
        STRAFE_DIRECTION_FOR_PARKING = -1;
        vision.CENTER_TAG_ID = 5;

    }

    //------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize
        robot.init(hardwareMap);
        vision.initDoubleVision(hardwareMap);
        setUniqueParameters();


        while (!isStarted()) {
            if (opModeInInit()) {
                vision.detectTeamElement(); // run detections continuously.
                telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                vision.telemetryAprilTag();
                telemetry.update();
            }
        }


        waitForStart();

        while (opModeIsActive()) {
            switch (currentStage) {
                case DETECT_TE:
                    vision.detectTeamElement();
                    telemetry.addData("Target Tag ID", vision.TARGET_TAG_ID);
                    telemetry.update();
                    currentStage = AutoStages.OUTTAKE;
                    break;
                case OUTTAKE:
                    /**
                     * Step 2: Deliver purple pixel to the detected Position.
                     * (common to all auto)
                     */

                    switch (vision.TARGET_SPIKE_MARK) {
                        case 1:
                            outtakeToMark1And4();
                            break;
                        case 2:
                            outtakeToMark2And5();
                            break;
                        case 3:
                            outtakeToMark3And6();
                            break;
                    }
                    currentStage = AutoStages.GOTO_BACKBOARD;
                    break;
                case GOTO_BACKBOARD:
                    /**
                     * Step 3: Drive to Backstage.
                     *
                     */
                    sleep(4000);
                    gotoBackBoard(STRAFE_TO_BACKBOARD_DIRECTION, STRAFE_TO_BACKBOARD_DISTANCE, TURN_ANGLE_TO_FACE_BACKBOARD, STRAFE_TO_MIDDLE_TAG_DISTANCE);
                    currentStage = AutoStages.CENTER_AprilTag;
                    break;
                case CENTER_AprilTag:
                    centerToCenterTag();
                    currentStage = AutoStages.DELVER_BACKBOARD_PARK;
                    break;
                case DELVER_BACKBOARD_PARK:
                    deliverToBackboardAndPark();
                    sleep(1000);
                    currentStage = AutoStages.END_AUTO;
                    break;
                case END_AUTO:
                    // End Auto keeps printing debug information via telemetry.
                    telemetry.update();
                    sleep(5000); //5 sec delay between telemetry.
            }

        } // end while loop

    }  //end opMode

    /**
     * Methods for driving the robot.
     */
    public void outtakeToMark2And5() throws InterruptedException {
        if(USE_NEW_PATH) {
            robot.chassis.Drive(DRIVE_SPEED, 23);
            robot.chassis.autoTurn(175, TURN_OFFSET);
            robot.intake.MoveIntake(0.4, true);
            Thread.sleep(2000);
            robot.intake.MoveIntake(0, true);
            robot.chassis.Drive(DRIVE_SPEED, 23);
        } else {
            robot.chassis.Drive(DRIVE_SPEED, 48);
            robot.intake.MoveIntake(0.4, true);
            Thread.sleep(2000);
            robot.intake.MoveIntake(0, true);
            robot.chassis.Drive(DRIVE_SPEED, 2);
        }
    }

    public void outtakeToMark3And6() throws InterruptedException {
        if(USE_NEW_PATH) {
            robot.chassis.Drive(DRIVE_SPEED, 27);
            robot.chassis.autoTurn(-100, TURN_OFFSET);
            robot.chassis.Drive(DRIVE_SPEED, 3);
            robot.intake.MoveIntake(0.4, true);
            Thread.sleep(2000);
            robot.intake.MoveIntake(0, true);
            robot.chassis.Drive(DRIVE_SPEED, -2);
            robot.chassis.Strafe(DRIVE_SPEED, -23);
            robot.chassis.autoTurn(-100, TURN_OFFSET);
        } else {
            robot.chassis.Drive(DRIVE_SPEED, 27);
            robot.chassis.autoTurn(-93, TURN_OFFSET);
            robot.chassis.Drive(DRIVE_SPEED, 3);
            robot.intake.MoveIntake(0.4, true);
            Thread.sleep(2000);
            robot.intake.MoveIntake(0, true);
            robot.chassis.Drive(DRIVE_SPEED, -2);
            robot.chassis.Strafe(DRIVE_SPEED, 30);
            robot.chassis.autoTurn(93, TURN_OFFSET);
        }
    }

    public void outtakeToMark1And4() throws InterruptedException {
        if(USE_NEW_PATH) {
            robot.chassis.Drive(DRIVE_SPEED, 27);
            robot.chassis.autoTurn(90, TURN_OFFSET);
            robot.chassis.Drive(DRIVE_SPEED, 3);
            robot.intake.MoveIntake(0.4, true);
            Thread.sleep(2000);
            robot.intake.MoveIntake(0, true);
            robot.chassis.Drive(DRIVE_SPEED, -3);
            robot.chassis.Strafe(DRIVE_SPEED, 21);
            robot.chassis.autoTurn(100, TURN_OFFSET);
        } else {
            robot.chassis.Drive(DRIVE_SPEED, 27);
            robot.chassis.autoTurn(93, TURN_OFFSET);
            robot.chassis.Drive(DRIVE_SPEED, 3);
            robot.intake.MoveIntake(0.4, true);
            Thread.sleep(2000);
            robot.intake.MoveIntake(0, true);
            robot.chassis.Drive(DRIVE_SPEED, -3);
            robot.chassis.Strafe(DRIVE_SPEED, -30);
            robot.chassis.autoTurn(-90, TURN_OFFSET);
        }
    }

    public void gotoBackBoard(int STRAFE_TO_BACKBOARD_DIRECTION, int STRAFE_TO_BACKBOARD_DISTANCE, int TURN_ANGLE_TO_FACE_BACKBOARD, int STRAFE_TO_MIDDLE_TAG_DISTANCE) throws InterruptedException {
        robot.chassis.Strafe(DRIVE_SPEED, STRAFE_TO_BACKBOARD_DIRECTION * STRAFE_TO_BACKBOARD_DISTANCE);
        robot.chassis.autoTurn(TURN_ANGLE_TO_FACE_BACKBOARD, TURN_OFFSET);
        robot.chassis.Strafe(DRIVE_SPEED, STRAFE_TO_MIDDLE_TAG_DISTANCE);
    }

    public void deliverToBackboardAndPark() {

        // Swing the arm and wist to low position.
        robot.arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.gotoAutoPosition();
        robot.wrist.gotoAutoPosition();
        sleep(1000);

        // Open the gate to deliver one pixel.
        robot.gate.open();
        sleep(2000);

        // Bring the wrist and arm to pickup position.
        robot.wrist.gotoPickupPosition();
        sleep(850);
        robot.arm.gotoPickupPosoition();

        // Park.
        robot.chassis.Strafe(DRIVE_SPEED, 3 - vision.TARGET_SPIKE_MARK * 6 + (20*STRAFE_DIRECTION_FOR_PARKING));
        robot.chassis.Drive(DRIVE_SPEED, 9);
    }


    public void centerToCenterTag() {
        double yawError = 0;
        float turnOffsetAprilTag = 0;
        double edgeOffset = 7;
        AprilTagDetection tempTag = null;
        vision.centerTag = vision.detect_apriltag(vision.CENTER_TAG_ID);

        if (vision.centerTag != null) {
            yawError = vision.centerTag.ftcPose.yaw;
            if (yawError != 0) {
                robot.chassis.autoTurn((float) -yawError, turnOffsetAprilTag);
                sleep(500);
            }
        }

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        // turn = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


        tempTag = vision.detect_apriltag(vision.CENTER_TAG_ID);
        if (tempTag != null) {
            vision.centerTag = tempTag; //Update the center tag if detection was successful.
        }

        double strafeError = vision.centerTag.ftcPose.x;
        if (vision.TARGET_TAG_ID < vision.CENTER_TAG_ID) {
            strafeError -= edgeOffset;
        } else if (vision.TARGET_TAG_ID > vision.CENTER_TAG_ID) {
            strafeError += edgeOffset;
        }

        robot.chassis.Strafe(DRIVE_SPEED, strafeError); //x is in inches.
        sleep(500);


        tempTag = vision.detect_apriltag(vision.TARGET_TAG_ID);
        if (tempTag != null) {
            vision.centerTag = tempTag;
        }
        double rangeError = vision.centerTag.ftcPose.y / 2.54 - vision.DELIVERY_DISTANCE;

        robot.chassis.Drive(DRIVE_SPEED * 0.5, (float) rangeError);


        sleep(500);

    }


}