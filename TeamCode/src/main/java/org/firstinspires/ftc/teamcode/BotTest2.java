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

    public void init(HardwareMap map, Telemetry tele) {
        this.map = map;
        this.tele = tele;

        //Motors from hardware map
        FL = map.get(DcMotor.class, "FL");
        FR = map.get(DcMotor.class, "FR");
        BL = map.get(DcMotor.class, "BL");
        BR = map.get(DcMotor.class, "BR");
        lift = map.get(DcMotor.class, "lift");
        //jewelServo = map.get(Servo.class, "jewel");
        leftClamp = map.get(Servo.class, "lclamp");
        rightClamp = map.get(Servo.class, "rclamp");

        //Setting runmode
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set direction for drive
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

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
