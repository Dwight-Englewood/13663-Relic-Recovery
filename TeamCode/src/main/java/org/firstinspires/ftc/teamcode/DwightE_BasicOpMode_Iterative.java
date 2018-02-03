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
    BotTest2 robot = new BotTest2();
    double jPos = 0.5;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        robot.setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if(gamepad1.x || gamepad1.right_trigger > .2) jPos += 0.1;
        if(gamepad1.y || gamepad1.left_trigger > .2) jPos -=0.1;




        robot.jewelServo.setPosition(jPos);
        telemetry.addData("ARGB: ",robot.cSensor.argb() );
        telemetry.addData("BLUE:  ", robot.cSensor.blue());
        telemetry.addData("RED: ", robot.cSensor.red());
        telemetry.addData("GREEN ", robot.cSensor.green());
        telemetry.addData("Jewel Servo: ", robot.jewelServo.getPosition());

        // Show the elapsed game time and wheel power.

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}