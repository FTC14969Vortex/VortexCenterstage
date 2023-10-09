package org.firstinspires.ftc.teamcode.OpModes;
//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "23-24 TeleOp", group = "TeleOp")

public class Teleop extends LinearOpMode {

    //tells you how long the robot has run for
//    private ElapsedTime runtime = new ElapsedTime();
//    double timeout_ms = 0;

    Robot robot = new Robot();

    //How fast your robot will accelerate.
    public double acceleration = 0.3;


    //Motor powers
    public double fl_power = 0;
    public double bl_power = 0;
    public double fr_power = 0;
    public double br_power = 0;

    public double DRIVETRAIN_SPEED = 0.7;

    //ArmWrist
    public void initDeliver() {
        robot.arm.swingUp();
        robot.wrist.Deliver();
    }
    public void initPick() {
        robot.arm.swingDown();
        robot.wrist.Pick();
    }




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

        telemetry.addData("FL Motor Encoder", robot.chassis.FLMotor.getCurrentPosition());
        telemetry.addData("BL Motor Encoder", robot.chassis.BLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Encoder", robot.chassis.BRMotor.getCurrentPosition());
        telemetry.addData("FR Motor Encoder", robot.chassis.FRMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {
            // Mapping from Xbox series S controller to motor powers.
            double move_y_axis = gamepad1.left_stick_y;
            double move_x_axis = -gamepad1.left_stick_x;
            double pivot_turn = -gamepad1.right_stick_x;


            //Sets the target power
            double target_fl_power = move_y_axis + move_x_axis + pivot_turn;
            double target_bl_power = move_y_axis - move_x_axis + pivot_turn;
            double target_fr_power = move_y_axis - move_x_axis - pivot_turn;
            double target_br_power = move_y_axis + move_x_axis - pivot_turn;

            //Adds how far you are from target power, times acceleration to the current power.
            fl_power += acceleration * (target_fl_power - fl_power);
            bl_power += acceleration * (target_bl_power - bl_power);
            fr_power += acceleration * (target_fr_power - fr_power);
            br_power += acceleration * (target_br_power - br_power);

            /**
             Joystick controls for Drivetrain
             */

            robot.chassis.FLMotor.setPower(DRIVETRAIN_SPEED * fl_power);
            robot.chassis.BLMotor.setPower(DRIVETRAIN_SPEED * bl_power);
            robot.chassis.FRMotor.setPower(DRIVETRAIN_SPEED * fr_power);
            robot.chassis.BRMotor.setPower(DRIVETRAIN_SPEED * br_power);

            /**
             * Joystick controls for slider, arm, and claw.
             */

            //Intake
            if (gamepad1.left_bumper) {
                robot.intake.MoveIntake(0.8, true);
            }
            if (gamepad1.right_bumper) {
                robot.intake.MoveIntake(0.8, false);
            }
            if (gamepad1.x) {
                robot.intake.motor.setPower(0);

            }
            //Claw
//            if (gamepad2.x) {
//                robot.claw.open();
//            }
//            if (gamepad2.y) {
//                robot.claw.close();
//            }
//            //Arm delivery
//            if (gamepad2.right_bumper) {
//                initDeliver();
//            }
//            if (gamepad2.left_bumper) {
//                initPick();
//            }
            //double wristPosition = -gamepad2.left_stick_x;
            //double armPower = -gamepad2.right_stick_y;
           // robot.wrist.servo.setPosition(wristPosition);
           // robot.arm.motor.setPower(armPower);

        }




            telemetry.addData("FL Motor Encoder", robot.chassis.FLMotor.getCurrentPosition());
            telemetry.addData("BL Motor Encoder", robot.chassis.BLMotor.getCurrentPosition());
            telemetry.addData("BR Motor Encoder", robot.chassis.BRMotor.getCurrentPosition());
            telemetry.addData("FR Motor Encoder", robot.chassis.FRMotor.getCurrentPosition());
            org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
            angle = robot.chassis.imu.getAngularOrientation();
            telemetry.addData("Angular Orientation", angle);
            int angleFloat = (int) (robot.modAngle(angle.firstAngle));
            telemetry.addData("Orientation in 0-360", angleFloat);
            telemetry.addData("Robot Location", "(" + robot.chassis.robotX + ", " + robot.chassis.robotY + ")");
            telemetry.addData("IsRobotStable", robot.chassis.isRobotStable());

            telemetry.update();

        }

    }