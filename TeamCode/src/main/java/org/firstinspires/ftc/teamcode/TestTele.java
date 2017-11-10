package org.firstinspires.ftc.teamcode;

/**
 * Created by Miracm on 11/10/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Bot;



@TeleOp(name="TestTele", group="Iterative Opmode")

public class TestTele extends OpMode{
    Bot robot = new Bot(hardwareMap, telemetry);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void init_loop(){};

    @Override
    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
    //Add stuff here. Blah.
    }
    //Something- We might need some movement Enums
    @Override
    public void stop() {
        robot.drive(0);
    }
}




