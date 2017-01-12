package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name="HTTest",group = "Tests")
public class HTTest extends OpMode {
    private ColorSensor light;
    private boolean led, toggle;
    @Override
    public void init() {
        light = hardwareMap.colorSensor.get("color");
        light.enableLed(false);
    }

    @Override
    public void loop() {
        if(gamepad1.y & !toggle){
            led = !led;
            toggle = true;
        } else if(!gamepad1.y) toggle = false;

        light.enableLed(led);
        telemetry.addData("Red: ", light.red());
        telemetry.addData("Blue: ", light.blue());
        telemetry.addData("Green: ", light.green());
    }
}
