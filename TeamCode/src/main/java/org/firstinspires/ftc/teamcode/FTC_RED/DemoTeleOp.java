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
@TeleOp(name="DemoOp",group="OpModes")
public class DemoTeleOp extends MainTeleOp {

    @Override
    protected void telemetry(){
            telemetry.addData("Flywheel is ", (isOn[0] ? "on. " : "off."));
            telemetry.addData("Intake is ", (isOn[1] ? "on. " : "off."));
            telemetry.addData("Belt is ", (isOn[2] ? "on. " : "off."));
            telemetry.addData("Left motor power: ", driveTrain.getGamepadYValues()[1] + driveTrain.getGamepadXValues()[1]);
            telemetry.addData("Right motor power: ", driveTrain.getGamepadYValues()[1] - driveTrain.getGamepadXValues()[1]);
            telemetry.addData("Flywheel power :", flyPower);
    }

    @Override
    public void loop() {
        driveTrain.arcadeDrive();
        checkGamepad1();
        checkGamepad2();
        telemetry();
    }


}
