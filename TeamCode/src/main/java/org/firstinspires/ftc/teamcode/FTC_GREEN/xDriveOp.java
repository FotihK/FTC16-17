package org.firstinspires.ftc.teamcode.FTC_GREEN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.DriveTrain;

/**
 * Created by HP on 12/30/2016.
 */
@TeleOp(name="ZiyanOp", group ="TeleOps")
public class xDriveOp extends OpMode {

    private DcMotor one, two, three, four;

    @Override
    public void init() {
        one = hardwareMap.dcMotor.get("one");
        two = hardwareMap.dcMotor.get("two");
        three = hardwareMap.dcMotor.get("three");
        four = hardwareMap.dcMotor.get("four");

    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.right_stick_y) > 0.05 && Math.abs(gamepad1.right_stick_x) > 0.05) {
            if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
                one.setPower(-gamepad1.right_stick_y);
                two.setPower(-gamepad1.right_stick_y);
                three.setPower(gamepad1.right_stick_y);
                four.setPower(gamepad1.right_stick_y);
            } else {
                one.setPower(-gamepad1.right_stick_x);
                two.setPower(gamepad1.right_stick_x);
                three.setPower(gamepad1.right_stick_x);
                four.setPower(-gamepad1.right_stick_x);
            }
        }
    }
}
