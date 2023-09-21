package org.firstinspires.ftc.teamcode.OpModes;

//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.IntakeHelper;
import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "Intake Test", group = "TeleOp")

public class IntakeTest extends LinearOpMode{

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.x) {
                robot.intake.MoveIntake(1, true, 5000);
            }
            if(gamepad1.y){
                robot.intake.motor.setPower(0);
            }

            if(gamepad1.a){
                robot.intake.MoveIntake(1, false, 5000);


            }
        }
    }
}
