package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name="ColorTest",group = "Tests")
public class ColorTest extends OpMode {
    private ColorSensor color;
    @Override
    public void init() {
        color = hardwareMap.colorSensor.get("color");
    }

    @Override
    public void loop() {
        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Green: ", color.green());
        telemetry.addData("Alpha: ", color.alpha());
        telemetry.addData("ARGB: ", color.argb());
        telemetry.update();
    }
}
