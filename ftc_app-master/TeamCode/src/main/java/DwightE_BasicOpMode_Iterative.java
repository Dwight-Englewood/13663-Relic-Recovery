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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Eli can't kill me", group="Iterative Opmode")
//@Disabled
public class DwightE_BasicOpMode_Iterative extends OpMode
{
    // Declare Motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private Servo clampServo1 = null;
    private Servo clampServo2 = null;
    private Servo testServo = null;

   // private DcMotor armMotor = null;

    /**
     * clamp Servos = 180 deg.
     *
     * arm Servos = 360 deg
     * */
   /* private Servo clampServo1 = null;
    private Servo clampServo2 = null;
    private Servo armServo1 = null;
    private Servo armServo2 = null;
*/
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");

        //clampServo1 = hardwareMap.servo.get("armServo1");
        //clampServo2 = hardwareMap.servo.get("armServo2");
        //testServo = hardwareMap.servo.get("backServo");



        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);


    //    clampServo1.setDirection(Servo.Direction.FORWARD);
     //   clampServo1.setPosition(0);
       // clampServo2.setDirection(Servo.Direction.REVERSE);
        //clampServo2.setPosition(0);
        //testServo.setPosition(0);



        // Tell the driver that initialization is complete.
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        //double leftPower1;
        //double rightPower1;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        boolean clamp = gamepad2.a;
        boolean back = gamepad2.b;
        leftPower    = Range.clip(drive + turn, -0.75, 0.75) ;
        rightPower   = Range.clip(drive - turn, -0.75, 0.75) ;
      //leftPower1    = Range.clip(drive + turn, -1.0, 1.0) ;
      // rightPower1   = Range.clip(drive - turn, -1.0, 1.0) ;
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftDrive1.setPower(leftPower);
        rightDrive1.setPower(rightPower);
  /*      if(gamepad2.a)
        {
            clampServo1.setPosition(.5);
            clampServo2.setPosition(.5);
            // .setPosition --> 0-1
            //clampServo1.setPosition();
            // clampServo2.setPosition();
        } else {
            clampServo1.setPosition(0.0);
            clampServo2.setPosition(0.0);
            telemetry.addData("A is not", "Pressed");
        }
        telemetry.update();

        if(gamepad2.b) {
            testServo.setPosition(.5);
        } else {
            testServo.setPosition(0.0);
            telemetry.addData("B is not", "Pressed");
        }
        telemetry.update();
*/

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
