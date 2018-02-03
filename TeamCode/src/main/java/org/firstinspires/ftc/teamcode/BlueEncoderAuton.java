package org.firstinspires.ftc.teamcode;

/**
 * Created by Kanet on 2/2/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.MovementEnum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Enums.MovementEnum;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Kanet on 2/1/2018.
 */
@Autonomous(name = "IWANSJ GAVE ME A B+ LOL(BLUE ENCODERS)",group = "Iterative OpMode")
public class BlueEncoderAuton extends OpMode {
    BotTest2 robot = new BotTest2();
    private ElapsedTime runtime = new ElapsedTime();
    int commandNum = 0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        robot.setDriveMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (commandNum) {
            // TODO: Check the weird ratios and put them below

            //FR: slowest
            //FL: fast
            //BR: slower
            //BL: fast
            case 0:

                robot.jewelServo.setPosition(0.99);
                if (robot.cSensor.blue() < 200) {
                    while(runtime.nanoseconds() < 200) {
                        robot.drive(MovementEnum.FORWARD, 0.2);
                    }
                } else {
                    while(runtime.nanoseconds() < 200) {
                        robot.drive(MovementEnum.BACKWARD, 0.2);
                    }
                }
                commandNum++;
                break;
            case 1:
                robot.setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.jewelServo.setPosition(0.5);
                commandNum++;
                break;
            case 2:
                robot.setDriveMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
                robot.FL.setTargetPosition(robot.distanceToRevs(0.5));
                robot.FR.setTargetPosition(robot.distanceToRevs(0.5));
                robot.BR.setTargetPosition(robot.distanceToRevs(0.5));
                robot.BL.setTargetPosition(robot.distanceToRevs(0.5));
                if(robot.FL.getTargetPosition() != robot.distanceToRevs(0.5)) {
                    commandNum--;
                }
                else commandNum++;

                break;
            case 3:
                robot.FL.setPower(0.25);
                robot.FR.setPower(-0.25);
                robot.BR.setPower(0.25);
                robot.BL.setPower(-0.25);
                telemetry.addData("Target Posisition: ", robot.FR.getTargetPosition());
                telemetry.addData("robot Power FL:", robot.FL.getPower());
                telemetry.addData("robot Power FR:", robot.FR.getPower());
                telemetry.addData("robot Power BL:", robot.BL.getPower());
                telemetry.addData("robot Power BR:", robot.BR.getPower());
                break;
        }

    }
}