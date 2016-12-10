package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name = "PrafulBot", group = "TeleOp")
public class TeleOp_PrafulBot extends OpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor /*lift,*/ intake, elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip /*latch,*/, load;
    private boolean[] toggleStates = new boolean[3]; //ind 0 is flywheel, ind 1 is flip servo, ind 2 is load servo
    private double flyPower = 0;
    private double flipPos = 0, loadPos = 0;

    @Override
    public void init() {
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        //lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");
        flip = hardwareMap.servo.get("flip");
        //latch = hardwareMap.servo.get("latch");
        load = hardwareMap.servo.get("load");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);
        //latch.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(flipPos);
        //latch.setPosition(0);
        load.setPosition(loadPos);
    }

    private double[] controllerYValues(Gamepad gamepad) {  // index 0 is left, index 1 is right
        double[] returnVals = new double[2];
        returnVals[0] = Math.abs(gamepad.left_stick_y) > 0.025 ? gamepad1.left_stick_y : 0;
        returnVals[1] = Math.abs(gamepad.right_stick_y) > 0.025 ? gamepad1.right_stick_y : 0;
        return returnVals;
    }

    public void checkGamepad1() {
        //5 and 7 for lift, up/down for buttons, 4 for toggle flywheel
        /*
        if(gamepad1.left_bumper){
            lift.setPower(0.95);
        } else if(gamepad1.right_bumper) {
            lift.setPower(-0.95);
        } else lift.setPower(0);
        */

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

        if (gamepad1.dpad_up) {
            intake.setPower(0.95);
        } else if (gamepad1.dpad_down) {
            intake.setPower(-0.95);
        } else intake.setPower(0);

        if (gamepad1.y && !toggleStates[0]) {
            flyPower = flyPower == 0 ? 0.9 : 0;
            flywheel.setPower(flyPower);
            toggleStates[0] = true;
        } else if (!gamepad1.y) toggleStates[0] = false;

        //if(gamepad1.start) latch.setPosition(1);
    }

    public void checkGamepad2() {
        //up and down for intake, 4 and 2 for elevator, 1 for flip servo

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

    public void telemetryUpdate() {
        telemetry.addData("Load Servo is on: ", loadPos == 0.5);
        telemetry.addData("Flip Servo is on: ", flipPos == 1);
    }

    @Override
    public void loop() {
        fL.setPower(controllerYValues(gamepad1)[0]);
        bL.setPower(controllerYValues(gamepad1)[0]);
        fR.setPower(controllerYValues(gamepad1)[1]);
        bR.setPower(controllerYValues(gamepad1)[1]);

        checkGamepad1();
        checkGamepad2();

        telemetryUpdate();
    }
}
