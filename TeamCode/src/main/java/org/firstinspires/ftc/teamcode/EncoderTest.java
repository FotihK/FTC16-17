package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by fotih on 11/11/2016.
 */
@Autonomous(name="Encoder Test", group="Tests")
public class EncoderTest extends LinearOpMode {
    private DcMotor fL, fR, bL, bR;
    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bL.setPower(0.4);
        bR.setPower(0.4);
        fL.setPower(0.4);
        fR.setPower(0.4);

        sleep(1000);

        bL.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        fR.setPower(0);

        telemetry.addData("Left Encoder Count: ", bL.getCurrentPosition());
        telemetry.addData("Right Encoder Count: ", bR.getCurrentPosition());
        telemetry.update();

        sleep(8000);


        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bL.setPower(-0.4);
        bR.setPower(-0.4);
        fL.setPower(-0.4);
        fR.setPower(-0.4);

        sleep(2000);

        bL.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        fR.setPower(0);

        telemetry.addData("Left Encoder Count: ", bL.getCurrentPosition());
        telemetry.addData("Right Encoder Count: ", bR.getCurrentPosition());
        telemetry.update();

        sleep(8000);
    }
}
