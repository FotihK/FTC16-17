package org.firstinspires.ftc.teamcode.FTC_RED.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("ALL")
@TeleOp(name = "Color", group = "Tests")

public class ColorTest extends OpMode {

    private ColorSensor color;
    private boolean toggle, led;
    private Servo pushL, pushR;
    private final double[] servoStartPositions = {0.94, 0.00};    //Left, Right
    private final double[] servoEndPositions = {0.55, 0.35};      //Left, Right

    @Override
    public void init() {
        color = hardwareMap.colorSensor.get("color");
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        pushL.setPosition(servoEndPositions[0]);
        pushR.setPosition(servoEndPositions[1]);

    }

    @Override
    public void loop() {
        telemetry.addData("ARGB: ", color.argb());
        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Green: ", color.green());
        telemetry.addData("Alpha: ", color.alpha());
        if(gamepad1.y && !toggle){
            led = !led;
            color.enableLed(led);
            toggle = true;
        } else if(!gamepad1.y) toggle = false;
    }
}
