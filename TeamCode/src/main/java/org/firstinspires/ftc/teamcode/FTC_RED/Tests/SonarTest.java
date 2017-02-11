package org.firstinspires.ftc.teamcode.FTC_RED.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp(name = "Sonar", group = "Tests")
public class SonarTest extends OpMode {

    private UltrasonicSensor sonar;

    @Override
    public void init() {
        sonar = hardwareMap.ultrasonicSensor.get("sonar");

    }

    @Override
    public void loop() {
        telemetry.addData("Sonar val: ", sonar.getUltrasonicLevel());
    }
}
