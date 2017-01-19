package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name = "PrafulBot", group = "TeleOp")
public class TeleOp_PrafulBot extends OpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor intake, elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip, load;
    private boolean[] toggleStates = new boolean[3]; //ind 0 is intake, ind 1 is flip servo, ind 2 is load servo
    private boolean movingState;
    private double flyPower = 0;
    private double flipPos = 0, loadPos = 0;
    private ElapsedTime rampTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");
        flip = hardwareMap.servo.get("flip");
        load = hardwareMap.servo.get("load");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(flipPos);
        load.setPosition(loadPos);
    }

    private void rampFlywheelUp() {
        if (flyPower <= 1.00 && rampTimer.time() > 350) {
            flyPower += 0.25;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        }
        if (flyPower >= 1.00) {
            flyPower = 1.00;
        }
        flywheel.setPower(flyPower);
    }

    private void rampFlywheelDown() {
        if (flyPower > 0 && rampTimer.time() > 450) {
            flyPower -= 0.25;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        } else if (flyPower <= 0) {
            flyPower = 0;
        }
        flywheel.setPower(flyPower);
    }

    private double[] controllerYValues(Gamepad gamepad) {  // index 0 is left, index 1 is right
        double[] returnVals = new double[2];
        returnVals[0] = Math.abs(gamepad.left_stick_y) > 0.025 ? gamepad1.left_stick_y : 0;
        returnVals[1] = Math.abs(gamepad.right_stick_y) > 0.025 ? gamepad1.right_stick_y : 0;
        return returnVals;
    }

    private void checkGamepad1() {
        if (gamepad1.dpad_left) {
            pushL.setPower(0.7);
            pushR.setPower(0.7);
        } else if (gamepad1.dpad_right) {
            pushL.setPower(-0.7);
            pushR.setPower(-0.7);
        } else {
            pushL.setPower(0);
            pushR.setPower(0);
        }

        if (gamepad1.x && !toggleStates[0]) {       //Up toggle for intake
            if (movingState) {
                intake.setPower(0);
                movingState = false;
            } else {
                movingState = true;
                intake.setPower(1.00);
            }
            toggleStates[0] = true;
        } else if (gamepad1.b && !toggleStates[0]) {    //Down toggle for intake
            if (movingState) {
                intake.setPower(0);
                movingState = false;
            } else {
                movingState = true;
                intake.setPower(-1.00);
            }
            toggleStates[0] = true;
        } else if (!gamepad1.dpad_down && !gamepad1.dpad_up) toggleStates[0] = false;

        if(gamepad1.y) rampFlywheelUp();
        if(gamepad1.a) rampFlywheelDown();
    }

    private void checkGamepad2() {
        if (gamepad2.dpad_up) {
            elevator.setPower(0.95);
        } else if (gamepad2.dpad_down) {
            elevator.setPower(-0.95);
        } else elevator.setPower(0);

        if (gamepad2.b && !toggleStates[1]) {
            flipPos = flipPos == 0 ? 0.5 : 0;
            flip.setPosition(flipPos);
            toggleStates[1] = true;
        } else if (!gamepad2.b) toggleStates[1] = false;

        if (gamepad2.x && !toggleStates[2]) {
            loadPos = loadPos == 0 ? 0.5 : 0;
            load.setPosition(loadPos);
            toggleStates[2] = true;
        } else if (!gamepad2.x) toggleStates[2] = false;
    }

    public void telemetry() {
        telemetry.addData("Load Servo is: ", loadPos == 0.5 ? "Up" : "Down");
        telemetry.addData("Flip Servo is: ", flipPos == 0.5 ? "Up" : "Down");
    }

    @Override
    public void loop() {
        fL.setPower(controllerYValues(gamepad1)[0]);
        bL.setPower(controllerYValues(gamepad1)[0]);
        fR.setPower(controllerYValues(gamepad1)[1]);
        bR.setPower(controllerYValues(gamepad1)[1]);

        checkGamepad1();
        checkGamepad2();

        telemetry();
    }
}
