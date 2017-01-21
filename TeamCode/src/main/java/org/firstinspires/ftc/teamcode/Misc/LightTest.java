package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name="LightTest",group = "Tests")
public class LightTest extends OpMode {
    private LightSensor light;
    @Override
    public void init() {
        light = hardwareMap.lightSensor.get("light_ground");
        light.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("Light Value: ", light.getLightDetected());
        telemetry.addData("Raw Light: ", light.getRawLightDetected());
        telemetry.update();
    }
}
