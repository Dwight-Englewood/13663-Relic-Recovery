package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Enums.BotActions;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.MovementEnum;

/**
 * Created by 10A on 11/5/2017.
 *
 * This is a basic bot class in which basic functions and motors are initialized so we don't have to reinitialize all parts all the time.
 */

public class Bot {

    // Motor Variables
  DcMotor FR, BR, FL, BL, SquishyL, SquishyR, Arm;
  HardwareMap map;
  Telemetry tele;
  // Servos that are used
  Servo leftServo, rightServo;

  public void init(HardwareMap map, Telemetry tele){
    this.map = map;
    this.tele = tele;

    // Initialize motors for phones
    FL = this.map.get(DcMotor.class, "LD1");
    FR = this.map.get(DcMotor.class, "RD1");
    BL = this.map.get(DcMotor.class, "LD");
    BR = this.map.get(DcMotor.class, "RD");
    SquishyL = this.map.get(DcMotor.class, "SquishL");
    SquishyR = this.map.get(DcMotor.class, "SquishR");
    Arm = this.map.get(DcMotor.class, "Armothy");

    //Servo names for phone
    leftServo = map.servo.get("leftServo");
    rightServo = map.servo.get("rightServo");


      // Set Motor Run Modes
    FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    SquishyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    SquishyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //set motor Directions
    FR.setDirection(DcMotorSimple.Direction.REVERSE);
    FL.setDirection(DcMotorSimple.Direction.FORWARD);
    BR.setDirection(DcMotorSimple.Direction.REVERSE);
    BL.setDirection(DcMotorSimple.Direction.FORWARD);
    SquishyL.setDirection(DcMotorSimple.Direction.FORWARD);
    SquishyR.setDirection(DcMotorSimple.Direction.REVERSE);
    Arm.setDirection(DcMotorSimple.Direction.FORWARD);



    BR.setPower(0);
    BL.setPower(0);
    FR.setPower(0);
    FL.setPower(0);
    SquishyR.setPower(0);
    SquishyL.setPower(0);
    Arm.setPower(0);
    //tele.addData(">","Gyro Calibrating. Do Not move!");
    //tele.update();
  }

  public void tankDrive(double leftStick, double rightStick, double leftTrigger, double rightTrigger, boolean invert) {
    int i = invert ?  -1 : 1;

    if (leftTrigger > .3) {
      drive(MovementEnum.LEFTSTRAFE, leftTrigger * i);
      return;
    }
    if (rightTrigger > .3){
      drive(MovementEnum.RIGHTSTRAFE, rightTrigger * i);
      return;
    }
    leftStick *= i;
    rightStick *= i;

    FL.setPower(-leftStick);
    BL.setPower(-leftStick);
    FR.setPower(-rightStick);
    BR.setPower(-rightStick);
  }

  //TODO: DIAGONALS
  public void drive(MovementEnum movement, double power) {
    switch (movement){
      case FORWARD:
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
        break;

      case BACKWARD:
        FL.setPower(-power);
        FR.setPower(-power);
        BL.setPower(-power);
        BR.setPower(-power);
        break;

      case LEFTSTRAFE:
        FL.setPower(-power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(-power);
        break;

      case RIGHTSTRAFE:
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(-power);
        BR.setPower(power);
        break;

      case LEFTTURN:
        FL.setPower(-power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(power);
        break;

      case RIGHTTURN:
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(-power);
        break;

      case STOP:
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        break;
    }
  }


    /**
     * This is a really weird way to make it drive
     *
     * @param power = some gamepad value to let it drive forward
     */
    // TODO: change this for the freshman's "better" way to drive
    public void drive(double power) {
    FL.setPower(power);
    FR.setPower(power);
    BL.setPower (power);
    BR.setPower(power);
  }

    /**
     * This is a way to turn
     *
     * @param in = the value will be how fast you want to turn
     */
    // TODO: change this for the freshman's "better" way to turn
  public void turn(double in){
    BR.setPower(-in);
    FR.setPower(-in);
    BL.setPower(in);
    FL.setPower(in);
  }

    /**
     * This controls how much armothy, the linear slide, gets turned on
     *
     * @param up = how fast the linear slide will go up
     */
  public void armUp(double up){
      Arm.setPower(up);
  }

    /**
     * This is how fast you can make Armothy flacid.
     *
     * @param down = how fast you want to turn off armothy
     */
  public void armDown(double down){
    Arm.setPower(down);
  }

    /**
     * This is a thing for the intake
     *
     * @param nPower = how fast the compliant wheels will grab the blocks
     */
  public void putItInMe(double nPower){
      SquishyL.setPower(nPower);
      SquishyR.setPower(-nPower);
  }

    /**
     * This is how the clamp clamps.
     *
     */
  public void clampItDown() {
    leftServo.setPosition(0.5);
    rightServo.setPosition(0.5);
  }

    /**
     * This is how the clamp lets go
     */
  public void letGo() {
    leftServo.setPosition(0);
    rightServo.setPosition(0);
  }

  //Stuff
  public void intake(double power){
    intakeOne.setPower(power);
    intakeTwo.setPower(power);
  }

}
