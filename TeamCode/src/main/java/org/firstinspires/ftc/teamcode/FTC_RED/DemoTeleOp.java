package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.DriveTrain;


/**
 * Created by HP on 11/30/2016.
 */
@TeleOp(name = "DemoOp", group = "OpModes")
public class DemoTeleOp extends MainTeleOp {

    @Override
    protected void telemetry() {
        telemetry.addData("Flywheel is ", (isOn[0] ? "on. " : "off."));
        telemetry.addData("Intake is ", (isOn[1] ? "on. " : "off."));
        telemetry.addData("Belt is ", (isOn[2] ? "on. " : "off."));
        telemetry.addData("Left motor power: ", driveTrain.getGamepadYValues(gamepad1)[1] + driveTrain.getGamepadXValues(gamepad1)[1]);
        telemetry.addData("Right motor power: ", driveTrain.getGamepadYValues(gamepad1)[1] - driveTrain.getGamepadXValues(gamepad1)[1]);
        telemetry.addData("Flywheel power :", flyPower);
        if (isOn[3]) {
            telemetry.addData("Ground Sensor Raw/Detected: ", light_ground.getRawLightDetected() + "/" + light_ground.getLightDetected());
            telemetry.addData("Beacon Sensor Raw/Detected: ", light_beacon.getRawLightDetected() + "/" + light_beacon.getLightDetected());
        }
    }

    @Override
    public void loop() {
        driveTrain.arcadeDrive(gamepad1);
        checkGamepad1();
        checkGamepad2();
        telemetry();
    }


}
