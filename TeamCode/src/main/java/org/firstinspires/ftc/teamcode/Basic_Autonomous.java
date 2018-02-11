/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;


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
 * This will be the first Trial at Basic Autonomous programming.
 *
 * It will later be changed to the iterative opmode but for now we need a basic thing that will work
 */

@Autonomous(name="Basic Auton Anywhere(Blue TIMEDD)", group="Iterative Opmode")
//@Disabled
public class Basic_Autonomous extends OpMode {
    BotTest2 robot = new BotTest2();
    private ElapsedTime runtime = new ElapsedTime();
    int colour;
    int commandNum = -1;

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
        robot.jewelServo.setPosition(0.1);
        robot.cSensor.enableLed(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch(commandNum){
            // TODO: Check the weird ratios and put them below

            //FR: slowest
            //FL: fast
            //BR: slower
            //BL: fast
            case -2:
                if (runtime.milliseconds() >= 2000) {
                    robot.drive(MovementEnum.STOP, 0.0);
                } else {
                    robot.drive(MovementEnum.RIGHTTURN, 0.2);

                    break;

            /*
            case -1:
                robot.jewelServo.setPosition(0.8);
                runtime.reset();
                commandNum++;
                break;

            case 0:
             //   telemetry.addData("blue" robot.cSensor.blue());

                if (runtime.milliseconds() > 6000) {
                 commandNum++;

                } else if (runtime.milliseconds() > 1000) {
                    if (robot.cSensor.blue() > robot.cSensor.red()) {
                        robot.FR.setPower(0.1);
                        robot.BR.setPower(0.1);

                    } else if (robot.cSensor.blue() < robot.cSensor.red()) {
                        robot.FR.setPower(-0.1);
                        robot.BR.setPower(-0.1);

                    }
                }
                robot.FR.setPower(0.0);
                robot.BR.setPower(0.0);
                break;
            case 1:

              robot.setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                runtime.reset();
                robot.jewelServo.setPosition(0.3);
             commandNum++;
                break;

            case 2:
                robot.setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.jewelServo.setPosition(0.1);
                if(robot.jewelServo.getPosition() > 0.3) commandNum--;

                else {
                    if (runtime.milliseconds() >= 2000) {
                        robot.drive(MovementEnum.STOP, 0.0);
                    } else {
                        robot.drive(MovementEnum.RIGHTTURN, 0.42);
                    }


                    if (runtime.milliseconds() >= 2000) {
                        robot.drive(MovementEnum.STOP, 0.0);
                    } else {
                        robot.drive(MovementEnum.RIGHTTURN, 0.2);
                     */
                }
                telemetry.addData("Target Posisition: ", robot.FR.getTargetPosition());
                telemetry.addData("robot Power FL:", robot.FL.getPower());
                telemetry.addData("robot Power FR:", robot.FR.getPower());
                telemetry.addData("robot Power BL:", robot.BL.getPower());
                telemetry.addData("robot Power BR:", robot.BR.getPower());
                telemetry.addData("jewelServo", robot.jewelServo.getPosition());
                break;
        }
        telemetry.addData("Target Posisition: ", robot.FR.getTargetPosition());
        telemetry.addData("robot Power FL:", robot.FL.getPower());
        telemetry.addData("robot Power FR:", robot.FR.getPower());
        telemetry.addData("robot Power BL:", robot.BL.getPower());
        telemetry.addData("robot Power BR:", robot.BR.getPower());
        telemetry.addData("jewelServo", robot.jewelServo.getPosition());
        telemetry.addData("colour", robot.cSensor.blue());
        }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

