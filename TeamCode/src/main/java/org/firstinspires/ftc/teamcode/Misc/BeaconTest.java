package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name="BeaconTest",group = "Tests")
public class BeaconTest extends OpMode {
    private LightSensor light;
    private boolean led, toggle;
    @Override
    public void init() {
        light = hardwareMap.lightSensor.get("light_beacon");
        light.enableLed(false);
    }

    @Override
    public void loop() {
        if(gamepad1.y & !toggle){
            led = !led;
            toggle = true;
        } else if(!gamepad1.y) toggle = false;

        light.enableLed(led);
        telemetry.addData("Light Value: ", light.getLightDetected());
        telemetry.addData("Raw Light: ", light.getRawLightDetected());
        telemetry.update();
    }
}
