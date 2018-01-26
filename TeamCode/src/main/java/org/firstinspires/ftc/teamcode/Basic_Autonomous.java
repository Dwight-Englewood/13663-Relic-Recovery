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

@Autonomous(name="Basic Auton Anywhere(Blue)", group="Iterative Opmode")
//@Disabled
public class Basic_Autonomous extends OpMode {
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
        switch(commandNum){
            // TODO: Check the weird ratios and put them below

            //FR: slowest
            //FL: fast
            //BR: slower
            //BL: fast
            case 0:
                robot.setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.FL.setPower(0.25);
                robot.BL.setPower(-0.25);
                robot.FR.setPower(0.25);
                robot.BR.setPower(-0.25);
                while (runtime.milliseconds() < 1200){}
                telemetry.addData("robot Power FL:",robot.FL.getPower() );
                // telemetry.addData("robot Power FR:",robot.FR.getPower());
                //telemetry.addData("robot Power BL:",robot.BL.getPower());
                //telemetry.addData("robot Power BR:",robot.BR.getPower());
                break;
        /*

            case -1:
                robot.jewelServo.setPosition(1.0);
                if(robot.colourSensor.blue() < 2.0){
                    robot.drive(MovementEnum.FORWARD, 0.2);
                }
                else robot.drive(MovementEnum.BACKWARD, 0.2);
                commandNum++;
                break;
            case 0:
                robot.setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                commandNum++;
                break;
            case 1:
                robot.FL.setTargetPosition(10);
                robot.FR.setTargetPosition(10);
                robot.BR.setTargetPosition(10);
                robot.BL.setTargetPosition(10);
                //robot.distanceToRevs(2.)
                if(robot.FL.getTargetPosition() != 10){commandNum--;}
                else commandNum++;
                break;
            case 2:
                robot.setDriveMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
                robot.BR.setPower(0.75);
                robot.FR.setPower(0.75);
                robot.BL.setPower(0.75);
                robot.FL.setPower(0.75);

                telemetry.addData("Target Encoder Position: ", robot.FL.getTargetPosition());
                telemetry.addData("Power to wheels:", robot.FL.getPower());
                telemetry.update();
                break;

*/

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

