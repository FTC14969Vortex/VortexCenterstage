package org.firstinspires.ftc.teamcode.OpModes;
//imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "CenterStageTeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {

    //tells you how long the robot has run for
//    private ElapsedTime runtime = new ElapsedTime();
//    double timeout_ms = 0;

    Robot robot = new Robot();

    //How fast your robot will accelerate.
    public double ACCELERATION = 0.3;

    double WRIST_HOLDING_POWER = 0.7;

    double ARM_HOLDING_POWER = 0.1;


    double INTAKE_SPEED = 1;


    int timeout_ms = 5000;

    //Motor powers
    public double fl_power = 0;
    public double bl_power = 0;
    public double fr_power = 0;
    public double br_power = 0;

    public double DRIVETRAIN_SPEED = 0.5;
    public double slider_position = 0;

//    //ArmWrist
//    public void DeliverPosition() {
//        robot.arm.swingUp();
//        //robot.arm.motor.setPower(armHoldingPower);
//        robot.wrist.Deliver();
//        //robot.wrist.servo.setPosition(WristHoldingPower);
//
//    }
//    public void DefaultPosition() {
//        robot.arm.motor.setPower(0);
//        robot.arm.swingDown();
//        robot.wrist.Default();
//        robot.wrist.servo.setPosition(0);
//    }

    @Override
    public void runOpMode() throws InterruptedException{
        /**
         * Instance of Robot class is initalized
         */
        robot.init(hardwareMap);

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */


        robot.chassis.FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.chassis.BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.chassis.FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.chassis.BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            /**
             * Joystick controls for Drivetrain, Intake on GAMEPAD 1
             */
            //Drive train

            //Speed Control
            if (gamepad1.a){
                DRIVETRAIN_SPEED = 0.5;
            }
            if (gamepad1.b){
                DRIVETRAIN_SPEED = 1;
            }

            // Controller to motor powers.
            double move_y_axis = gamepad1.left_stick_y;
            double move_x_axis = -gamepad1.left_stick_x;
            double pivot_turn = -gamepad1.right_stick_x;


            //Sets the target power
            double target_fl_power = move_y_axis + move_x_axis + pivot_turn;
            double target_bl_power = move_y_axis - move_x_axis + pivot_turn;
            double target_fr_power = move_y_axis - move_x_axis - pivot_turn;
            double target_br_power = move_y_axis + move_x_axis - pivot_turn;

            //Adds how far you are from target power, times acceleration to the current power.
            fl_power += ACCELERATION * (target_fl_power - fl_power);
            bl_power += ACCELERATION * (target_bl_power - bl_power);
            fr_power += ACCELERATION * (target_fr_power - fr_power);
            br_power += ACCELERATION * (target_br_power - br_power);


            robot.chassis.FLMotor.setPower(DRIVETRAIN_SPEED * fl_power);
            robot.chassis.BLMotor.setPower(DRIVETRAIN_SPEED * bl_power);
            robot.chassis.FRMotor.setPower(DRIVETRAIN_SPEED * fr_power);
            robot.chassis.BRMotor.setPower(DRIVETRAIN_SPEED * br_power);

            //Intake
            if (gamepad1.left_bumper) {
                robot.intake.MoveIntake(INTAKE_SPEED, true);
            }
            if (gamepad1.right_bumper) {
                robot.intake.MoveIntake(INTAKE_SPEED, false);
            }
            if (gamepad1.x) {
                robot.intake.motor.setPower(0);
            }




            /**
             * Joystick controls for Slider, Arm, Wrist, Gate on GAMEPAD 2
             */


            // Arm
            double swing_arm_power = gamepad2.left_stick_y * 0.6 + 0.05;
            // Running without encoder allows the arm to be swung from current position.
            robot.arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.motor.setPower(swing_arm_power);


            // Slider

            // Slider is driven by servo, which goes to absolute position.
            // Therefore we should update the slider_position (range: 0 to 1) with joystick input (range: -1 to 1).
            //slider_position  -= gamepad2.right_stick_y;
//            if(slider_position<0){
//                slider_position = 0;
//            }
//            if(slider_position>1){
//                slider_position = 1;
//            }
//            }
           // robot.slider.servo.setPosition(slider_position);



            //Gate
            if(gamepad2.cross){
                robot.gate.close();
            }
            if(gamepad2.circle){
                robot.gate.open();
            }
            if(gamepad2.square){
                robot.gate.middle();
            }

            // Set arm, wrist, and gate to pickup or delivery position with bumper.
            //Higher Delivery
            if(gamepad2.dpad_up){
                robot.gate.close();
                robot.chassis.stopDriveMotors();
                robot.arm.gotoHighPosition();
                Thread.sleep(150);
                robot.wrist.gotoHighPosition();

            }
            //Lower delivery
            if(gamepad2.dpad_left){
                robot.gate.close();
                robot.chassis.stopDriveMotors();
                robot.arm.gotoLowPosition();
                Thread.sleep(150);
                robot.wrist.gotoLowPosition();
            }
            //Pickup position
            if(gamepad2.dpad_down){
                robot.wrist.gotoPickupPosition();
                robot.gate.open();
                Thread.sleep(850);
                robot.chassis.stopDriveMotors();
                robot.arm.gotoPickupPosoition();
            }

            if(gamepad2.dpad_right) {
                robot.arm.gotoAutoPosition();
                Thread.sleep(100);
                robot.wrist.gotoAutoPosition();
                robot.gate.open();
                robot.chassis.stopDriveMotors();

            }

            if(gamepad1.y){
                double startPos = 0.3;
                double endPos = 0.15;
                double currPos = 0;
                double increment = -0.01;

                for (currPos = startPos; currPos > endPos; currPos = currPos + increment) {
                    robot.wrist.gotoPosition(currPos);
                    Thread.sleep(1000);
                }
            }

            if(gamepad2.left_bumper){
                robot.drone.Unlatched();
            }
            if(gamepad2.right_bumper) {
                robot.drone.Latched();
            }
            if(gamepad2.y) {
                robot.wrist.gotoPickupPosition();
            }



//          T E L E M E T R Y
            telemetry.addData("FL Motor Encoder", robot.chassis.FLMotor.getCurrentPosition());
            telemetry.addData("BL Motor Encoder", robot.chassis.BLMotor.getCurrentPosition());
            telemetry.addData("BR Motor Encoder", robot.chassis.BRMotor.getCurrentPosition());
            telemetry.addData("FR Motor Encoder", robot.chassis.FRMotor.getCurrentPosition());
            telemetry.addData("First Angle", robot.chassis.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Second Angle", robot.chassis.imu.getAngularOrientation().secondAngle);
            telemetry.addData("Third Angle", robot.chassis.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("Arm Position", robot.arm.motor.getCurrentPosition());
            telemetry.addData("Motor Status", robot.arm.motor.isBusy());
            telemetry.addData("Arm Power", robot.arm.motor.getPower());
            telemetry.addData("Wrist Position", robot.wrist.servo.getPosition());
            //telemetry.addData("Slider Position", robot.slider.servo.getPosition());
            telemetry.addData("Gate Position", robot.gate.servo.getPosition());
            telemetry.update();

        }




            telemetry.addData("FL Motor Encoder", robot.chassis.FLMotor.getCurrentPosition());
            telemetry.addData("BL Motor Encoder", robot.chassis.BLMotor.getCurrentPosition());
            telemetry.addData("BR Motor Encoder", robot.chassis.BRMotor.getCurrentPosition());
            telemetry.addData("FR Motor Encoder", robot.chassis.FRMotor.getCurrentPosition());
            org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
            angle = robot.chassis.imu.getAngularOrientation();
            telemetry.addData("Angular Orientation", angle);
            int angleFloat = (int) (robot.chassis.modAngle(angle.firstAngle));
            telemetry.addData("Orientation in 0-360", angleFloat);
            telemetry.addData("Robot Location", "(" + robot.chassis.robotX + ", " + robot.chassis.robotY + ")");
            telemetry.addData("IsRobotStable", robot.chassis.isRobotStable());

            telemetry.update();

        }

    }