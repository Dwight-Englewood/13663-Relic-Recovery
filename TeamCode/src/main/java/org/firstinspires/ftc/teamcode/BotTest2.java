package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.MovementEnum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Created by Miracm on 12/1/2017.
 */

public class BotTest2 {

    public DcMotor BL, BR, FL, FR, lift;
    //public Servo jewelServo;
    public Servo leftClamp, rightClamp;
    public HardwareMap map;
    public Telemetry tele;

    public BotTest2() {}


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
        leftServo = this.map.servo.get("leftServo");
        rightServo = this.map.servo.get("rightServo");
        leftServo2 = this.map.servo.get("leftServo_II");
        rightServo2 = this.map.servo.get("rightServo_II");
        jewelServo = this.map.servo.get("jewelServo");

        // Other things for phones
        colourSensor = this.map.get(ColorSensor.class,"colorSensor");




        //set motor Directions
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        SquishyL.setDirection(DcMotorSimple.Direction.FORWARD);
        SquishyR.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Servo Directions
        jewelServo.setDirection(Servo.Direction.FORWARD);


        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        SquishyR.setPower(0);
        SquishyL.setPower(0);
        Arm.setPower(0);
        //tele.addData(">","Gyro Calibrating. Do Not move!");
        //tele.update();

    public void tankDrive(double leftStick, double rightStick, double leftTrigger, double rightTrigger, boolean invert, boolean brake) {
        double i = invert ? -0.75:0.75;
        if (leftTrigger > .3) {
            drive(MovementEnum.LEFTSTRAFE, leftTrigger * i);
            return;
        }
        if (rightTrigger > .3) {
            drive(MovementEnum.RIGHTSTRAFE, rightTrigger *i);
            return;
        }
        leftStick *= i;
        rightStick *= i;

        FL.setPower(-leftStick);
        FR.setPower(-rightStick);
        BL.setPower(-leftStick);
        BR.setPower(-rightStick);
    }
    public void setPower(double power){
        FL.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        BR.setPower(power);
    }
    public void drive(MovementEnum movement, double power) {
        switch (movement) {
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
                FR.setPower(-power);
                BL.setPower(power);
                BR.setPower(power);
                break;

            case RIGHTSTRAFE:
                FL.setPower(power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(-power);
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

    public void setDriveMotorModes(DcMotor.RunMode mode) {
        FL.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        BR.setMode(mode);
    }
    public void setDriveZeroPowers(DcMotor.ZeroPowerBehavior behavior){
        FL.setZeroPowerBehavior(behavior);
        FR.setZeroPowerBehavior(behavior);
        BL.setZeroPowerBehavior(behavior);
        BR.setZeroPowerBehavior(behavior);
    }
    public int distanceToRevs(double distance) {
        final double wheelCirc = 31.9185813;

        final double gearMotorTickThing = .5 * 1120; //neverrest 40 = 1120,

        return (int) (gearMotorTickThing * (distance / wheelCirc));
    }
    //public void jewelUp(){jewelServo.setPosition(.6);}
    //public void jewelOut(){jewelServo.setPosition(.3);}
}
