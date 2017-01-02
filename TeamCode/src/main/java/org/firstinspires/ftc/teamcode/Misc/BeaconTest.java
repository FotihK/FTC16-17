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
    private LightSensor light2;
    @Override
    public void init() {
        light = hardwareMap.lightSensor.get("light_beacon");
        light2 = hardwareMap.lightSensor.get("light_ground");
        light.enableLed(false);
        light2.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("Light Value: ", light.getLightDetected());
        telemetry.addData("Raw Light: ", light.getRawLightDetected());
        telemetry.addData("Light2 Value: ", light2.getLightDetected());
        telemetry.addData("Raw2 Light: ", light2.getRawLightDetected());
        telemetry.update();
    }
}
