package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.MovementEnum;


/**
 * Created by Miracm on 12/1/2017.
 */
@TeleOp (name="TeleTest2", group="Teleop")
public class TeleTest2 extends OpMode {
    BotTest2 robot = new BotTest2();
    int countdown = 0;
    boolean i = false;
    boolean brakeToggle = false;
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        telemetry.clear();
        robot.jewelUp();
        robot.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, i, brakeToggle);

        if (gamepad1.left_bumper && countdown <= 0) {
            i = i ? false:true;
            countdown = 50;
        }

        if(gamepad1.right_bumper && countdown <=0) {
            brakeToggle = brakeToggle ? false:true;
        }

        if(gamepad2.x) {
            robot.jewelOut();
        } else {
            robot.jewelUp();
        }
        countdown --;
        telemetry.addData("Braking", brakeToggle);
        telemetry.update();
    }
    @Override
    public void stop() {
        robot.drive(MovementEnum.STOP, 0);
    }
}
