package org.firstinspires.ftc.teamcode.FTC_RED.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by HP on 12/30/2016.
 */
@Autonomous(name="Servo Test", group = "Tests")
public class ServoTest extends LinearOpMode {
    private Servo pushL, pushR;

    public void initialize() {
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        pushL.setPosition(0.5);
        pushR.setPosition(0.5);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                pushL.setPosition(Range.clip((pushL.getPosition() + 0.001), 0, 1));
            }
            if (gamepad1.b) {
                pushL.setPosition(Range.clip((pushL.getPosition() - 0.001), 0, 1));
            }

            if (gamepad1.x) {
                pushR.setPosition(Range.clip((pushR.getPosition() + 0.001), 0, 1));
            }
            if (gamepad1.y) {
                pushR.setPosition(Range.clip((pushR.getPosition() - 0.001), 0, 1));
            }

            telemetry.addData("L:", pushL.getPosition());
            telemetry.addData("R:", pushR.getPosition());
            telemetry.update();
        }
    }
}
