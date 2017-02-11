package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.TeleOpTemp;


/**
 * Created by HP on 11/30/2016.
 */
//@TeleOp(name = "DemoOp", group = "RED")
public class DemoTeleOp extends TeleOpTemp {

    @Override
    public void telemetry() {
        telemetry.addData("Flywheel is ", (isOn[0] ? "on. " : "off."));
        telemetry.addData("Intake is ", (isOn[1] ? "on. " : "off."));
        telemetry.addData("Belt is ", (isOn[2] ? "on. " : "off."));
        telemetry.addData("Left motor power: ", driveTrain.getGamepadYValues(gamepad1)[1] + driveTrain.getGamepadXValues(gamepad1)[1]);
        telemetry.addData("Right motor power: ", driveTrain.getGamepadYValues(gamepad1)[1] - driveTrain.getGamepadXValues(gamepad1)[1]);
        telemetry.addData("Flywheel power :", flyPower);
    }

    @Override
    public void loop() {
        driveTrain.arcadeDrive(gamepad1);
        checkGamepad1();
        checkGamepad2();
        telemetry();
    }


}
