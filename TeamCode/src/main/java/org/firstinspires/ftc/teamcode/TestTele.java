package org.firstinspires.ftc.teamcode;

/**
 * Created by Miracm on 11/10/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Bot;



@TeleOp(name="TestTele", group="Iterative Opmode")

public class TestTele extends OpMode{
    Bot robot = new Bot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void init_loop(){};

    @Override
    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
        //Doesn't this look rather familiar? :thinking:
        //robot.fieldCentricDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x); // Field centric?
        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, false); // Tank drive???



        if (gamepad2.right_bumper) {
            robot.intake(1);
        } else if (gamepad2.right_trigger > .3) {
            robot.intake(-1);
        } else {
            robot.intake(0);
        }


        if (gamepad2.right_stick_y > .3) {
            robot.intakeBrake.setPower(-1);
        } else if (gamepad2.right_stick_y < -.3) {
            robot.intakeBrake.setPower(1);
        } else {
            robot.intakeBrake.setPower(0);
        }

        if (gamepad2.b){
            robot.flipUp();
        } else {
            robot.flipDown();
        }

        if (gamepad2.dpad_up){
            robot.lift.setPower(-.5);
        } else if (gamepad2.dpad_down){
            robot.lift.setPower(1);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad2.y){
            robot.relLUp();
            robot.relRUp();
        } else if (gamepad2.a){
            robot.relLDown();
            robot.relRDown();
        }


    }
    //Something- We might need some movement Enums
    @Override
    public void stop() {
        robot.drive(0);
    }
}




