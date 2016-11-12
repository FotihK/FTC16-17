package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by fotih on 11/11/2016.
 */
@Autonomous(name = "Button Pushing", group = "Tests")
public class WorkBeaconWork extends LinearOpMode {
    private CRServo pushL, pushR;
    private LightSensor light;

    @Override
    public void runOpMode() throws InterruptedException {
        light = hardwareMap.lightSensor.get("light_beacon");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");

        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (light.getRawLightDetected() < 2.05) {  //Checks beacon and extends corresponding pusher
                pushL.setPower(0.5);
                sleep(1000);
                pushL.setPower(-0.5);
                sleep(500);
                pushL.setPower(0);
            } else {
                pushR.setPower(0.5);
                sleep(1000);
                pushR.setPower(-0.5);
                sleep(500);
                pushR.setPower(0);
            }

            sleep(7500);
        }
    }
}
