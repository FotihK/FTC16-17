package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by HP on 9/29/2016.
 */

@Autonomous(name="BeaconPushTest",group = "Tests")
public class BeaconPushTest extends LinearOpMode {
    private LightSensor light;
    private Servo pushL, pushR;

    public void initialize() {
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        pushL.setPosition(0.5);
        pushR.setPosition(0.5);
        light = hardwareMap.lightSensor.get("light_beacon");
        light.enableLed(false);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
