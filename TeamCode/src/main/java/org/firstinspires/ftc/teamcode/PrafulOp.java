package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name = "PrafulOp", group = "TeleOps")
public class PrafulOp extends OpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor lift, intake, elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip, latch;
    private boolean[] toggleStates = new boolean[2]; //ind 0 is flywheel, ind 1 is flip servo
    private double flyPower = 0;
    private final double flipStart = 0;

    @Override
    public void init() {
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");
        flip = hardwareMap.servo.get("flip");
        latch = hardwareMap.servo.get("latch");

        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.FORWARD);
        flip.setDirection(Servo.Direction.FORWARD);
        latch.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(flipStart);
        latch.setPosition(0);

    }

    private double[] controllerYValues(Gamepad gamepad) {  // index 0 is left, index 1 is right
        double[] returnVals = new double[2];
        returnVals[0] = Math.abs(gamepad.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        returnVals[1] = Math.abs(gamepad.right_stick_y) > 0.05 ? gamepad1.right_stick_y : 0;

        return returnVals;
    }

    public void checkGamepad1(){
        //5 and 7 for lift, up/down for buttons, 4 for toggle flywheel
        if(gamepad1.left_bumper){
            lift.setPower(0.95);
        } else if(gamepad1.right_bumper) {
            lift.setPower(-0.95);
        } else lift.setPower(0);

        if(gamepad1.dpad_up){
            pushL.setPower(0.7);
            pushR.setPower(0.7);
        } else if(gamepad1.dpad_down){
            pushL.setPower(-0.7);
            pushR.setPower(-0.7);
        } else {
            pushL.setPower(0);
            pushR.setPower(0);
        }

        if(gamepad1.y && !toggleStates[0]){
            flyPower = flyPower == 0 ? 0.95 : 0;
            flywheel.setPower(flyPower);
            toggleStates[0] = true;
        } else if(!gamepad1.y) toggleStates[0] = false;

        if(gamepad1.start) latch.setPosition(1);
    }

    public void checkGamepad2(){
        //up and down for elevator, 4 and 2 for lift, 1 for flip servo
        if(gamepad2.dpad_up){
            intake.setPower(0.95);
        } else if(gamepad2.dpad_down){
            intake.setPower(-0.95);
        } else intake.setPower(0);

        if(gamepad2.y){
            elevator.setPower(0.95);
        } else if(gamepad2.a){
            elevator.setPower(-0.95);
        } else elevator.setPower(0);

        if(gamepad1.x && !toggleStates[1]){
            flip.setPosition(flip.getPosition() == flipStart ? flipStart + 0.5 : flipStart);
            toggleStates[1] = true;
        } else if(!gamepad1.x) toggleStates[1] = false;
    }


    @Override
    public void loop() {
        fL.setPower(controllerYValues(gamepad1)[0]);
        bL.setPower(controllerYValues(gamepad1)[0]);
        fR.setPower(controllerYValues(gamepad1)[1]);
        bR.setPower(controllerYValues(gamepad1)[1]);

        checkGamepad1();
        checkGamepad2();



    }
}
