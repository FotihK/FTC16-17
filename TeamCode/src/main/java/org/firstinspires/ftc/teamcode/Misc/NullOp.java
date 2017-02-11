package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by HP on 9/29/2016.
 */

//@TeleOp(name="NullOp",group = "Tests")
public class NullOp extends OpMode {
    private long i;
    @Override
    public void init() {
        i = 1;
    }

    @Override
    public void loop() {
        telemetry.addData("Current Loop", i);
        telemetry.update();
        i++;
    }
}
