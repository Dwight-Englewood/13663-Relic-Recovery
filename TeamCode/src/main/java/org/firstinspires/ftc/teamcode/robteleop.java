package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.MovementEnum;

/**
 * Created by aburur on 1/6/18.
 */
@TeleOp(name="ROB TELEOP", group="TELEOP")
public class robteleop extends OpMode
{
    BotTest2 robot = new BotTest2();
    double rightPos = .5;
    double leftPos = .5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.tankDrive(gamepad1.left_stick_y, -1*gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, false, false);

        if (gamepad2.dpad_up) {
            robot.lift.setPower(.5);
        } else if (gamepad2.dpad_down) {
            robot.lift.setPower(-.5);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad2.right_trigger > .2) {
            rightPos -= .1;
        } else if (gamepad2.right_bumper) {
            rightPos += .1;
        }

        if (gamepad2.left_trigger > .2) {
            leftPos += .1;
        } else if (gamepad2.left_bumper) {
            leftPos -= .1;
        }

        robot.rightClamp.setPosition(rightPos);
        robot.leftClamp.setPosition(leftPos);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}