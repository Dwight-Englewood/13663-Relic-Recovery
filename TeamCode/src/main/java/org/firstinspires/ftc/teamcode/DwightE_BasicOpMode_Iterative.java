package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Enums.MovementEnum;

/**
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="A TeleOp for DAD", group="Iterative Opmode")
//@Disabled
public class DwightE_BasicOpMode_Iterative extends OpMode
{
   Bot robot = new Bot();
   boolean inverts = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        robot.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
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
       robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y,gamepad1.left_trigger,gamepad1.right_trigger, inverts);


        if(gamepad1.x) {
            robot.jewelServo.setPosition(0.5);
        }
        telemetry.update();

        if(gamepad1.a) {

            robot.leftServo.setPosition(0.5);
            robot.rightServo.setPosition(0.5);
            telemetry.addData("clamp", "is", "On");
        } else {
            if(gamepad1.b) {

                robot.leftServo.setPosition(0.0);
                robot.rightServo.setPosition(1.0);
                // telemetry.addData();
            }
            }
        telemetry.update();
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", MovementEnum.values());

        // Show the elapsed game time and wheel power.

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
