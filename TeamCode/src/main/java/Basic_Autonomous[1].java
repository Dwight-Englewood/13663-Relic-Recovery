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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This will be the first Trial at Basic Autonomous programming.
 *
 * @author TanayKane;
 *
 *
 * It will later be changed to the iterative opmode but for now we need a basic thing that will work
 */

@Autonomous(name="Attempt Uno at Auton", group="Linear Opmode")
//@Disabled
public class Basic_Autonomous extends LinearOpMode {

    // Declare Motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    //private DcMotor armJoint = null;
    //private DcMotor armBase = null;

   // private Servo clampServo1 = null;
    //private Servo clampServo2 = null;
    //private Servo jewelServo = null;

    ColorSensor colorSensor;
    private int position;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Hardware
        leftDrive  = hardwareMap.get(DcMotor.class, "LD");
        rightDrive = hardwareMap.get(DcMotor.class, "RD");
        leftDrive1  = hardwareMap.get(DcMotor.class, "LD1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "RD1");

       // jewelServo = hardwareMap.get(Servo.class, "jewel");

        /*
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);


        //jewelServo.setDirection(Servo.Direction.FORWARD);

        position = leftDrive.getCurrentPosition();
        //jewelServo.setPosition(0,0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // 1220 ticks per Revolution
        // 8 inches per rotation


        // jewelServo.setPosition(1.0)
        //if(colorSensor.blue()){
        //}
       // leftDrive.setTargetPosition(6754);
      //  leftDrive1.setTargetPosition(6754);
      //  rightDrive.setTargetPosition(6754);
       // rightDrive1.setTargetPosition(6754);

        leftDrive.setPower(0.2);
        leftDrive1.setPower(0.2);
        rightDrive.setPower(-0.2);
        rightDrive1.setPower(-0.2);

     //   jewelServo.setPosition(0);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (runtime.time() < 1.5) {

        }
        leftDrive.setPower(0.0);
        leftDrive1.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive.setPower(0.0);
    }
}
