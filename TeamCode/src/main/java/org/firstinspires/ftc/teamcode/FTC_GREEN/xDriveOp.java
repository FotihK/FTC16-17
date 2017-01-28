package org.firstinspires.ftc.teamcode.FTC_GREEN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by HP on 12/30/2016.
 */
@TeleOp(name = "ZiyanOp", group = "Green")
public class xDriveOp extends OpMode {

    private DcMotor one, two, three, four;

    @Override
    public void init() {
        one = hardwareMap.dcMotor.get("one");       //Negative + CW
        two = hardwareMap.dcMotor.get("two");       //Negative + CW
        three = hardwareMap.dcMotor.get("three");   //Positive + CCW
        four = hardwareMap.dcMotor.get("four");     //Positive + CCW

    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x) && Math.abs(gamepad1.right_stick_y) > 0.025) {
            one.setPower(-gamepad1.right_stick_y);
            two.setPower(-gamepad1.right_stick_y);
            three.setPower(gamepad1.right_stick_y);
            four.setPower(gamepad1.right_stick_y);
        } else if (Math.abs(gamepad1.right_stick_x) > 0.025) {
            one.setPower(-gamepad1.right_stick_x);
            two.setPower(gamepad1.right_stick_x);
            three.setPower(gamepad1.right_stick_x);
            four.setPower(-gamepad1.right_stick_x);
        } else if (Math.abs(gamepad1.left_stick_x) > 0.025) {
            one.setPower(-gamepad1.left_stick_x);
            two.setPower(-gamepad1.left_stick_x);
            three.setPower(-gamepad1.left_stick_x);
            four.setPower(-gamepad1.left_stick_x);
        } else {
            one.setPower(0);
            two.setPower(0);
            three.setPower(0);
            four.setPower(0);
        }
    }
}
