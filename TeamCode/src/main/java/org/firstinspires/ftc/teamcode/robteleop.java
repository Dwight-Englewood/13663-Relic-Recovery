package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.MovementEnum;

/**
 * Created by aburur on 1/6/18.
 */
@TeleOp(name="ROB TELEOP(DA REALIST 123)", group="TELEOP")
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
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setDriveZeroPowers(DcMotor.ZeroPowerBehavior.BRAKE);
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
        // TODO: update SDK and stuff off of google play store
        robot.tankDrive(gamepad1.left_stick_y, -1*gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, false, false);

        if (gamepad2.dpad_up) {
            robot.lift.setPower(.5);
        } else if (gamepad2.dpad_down) {
            robot.lift.setPower(-.5);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad2.right_trigger > .2) {
            rightPos -= .5;
            leftPos += .5;
        } else if (gamepad2.left_trigger > .2) {
            rightPos += .5;
            leftPos -= .5;
        }



        robot.rightClamp.setPosition(rightPos);
        robot.rightClamp2.setPosition(rightPos);
        robot.leftClamp2.setPosition(leftPos);
        robot.leftClamp.setPosition(leftPos);

        telemetry.addData("Front Left: ", robot.FL.getPower());
        telemetry.addData("Front Right: ", robot.FR.getPower());
        telemetry.addData("Back Left: ", robot.BL.getPower());
        telemetry.addData("Back Right: ", robot.BR.getPower());
        telemetry.addData("qwe6y7uojuul234rtyui: ", robot.jewelServo.getPosition());
        telemetry.addData("Servo L: ", robot.leftClamp.getPosition());
        telemetry.addData("Servo R: ", robot.rightClamp.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}}