package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by HP on 11/30/2016.
 */
@TeleOp(name="TeleOp",group="OpModes")
public class MainTeleOp extends OpMode {
    private DcMotor left, right;
    private DcMotor intake, belt, flywheelL, flywheelR;
    private Servo pushL, pushR;
    private boolean[] toggleStates = new boolean[5];    //0th is flywheel, 1st is intake, 2nd is belt, 3 is pushL, 4 is pushR
    private boolean[] movingStates = new boolean[2];   //0th is intake, 1st is belt
    private double flyPower = 0;
    private boolean flywheelOn = false;
    private ElapsedTime rampTimer = new ElapsedTime();

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        intake = hardwareMap.dcMotor.get("intake");
        belt = hardwareMap.dcMotor.get("belt");
        flywheelL = hardwareMap.dcMotor.get("flywheelL");
        flywheelR = hardwareMap.dcMotor.get("flywheelR");
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelL.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);

        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pushL.setPosition(0.9);
        pushR.setPosition(0.04);

    }

    private double[] controllerYValues(Gamepad gamepad) {  // index 0 is left, index 1 is right
        double[] returnVals = new double[2];
        returnVals[0] = Math.abs(gamepad.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        returnVals[1] = Math.abs(gamepad.right_stick_y) > 0.05 ? gamepad1.right_stick_y : 0;

        return returnVals;
    }

    private void rampFlywheelUp(){
        if(flyPower <= 0.95 && rampTimer.time() > 115) {
            flyPower += 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
        }
        if(flyPower == 0.95) flywheelOn = true;
        flywheelL.setPower(flyPower);
        flywheelR.setPower(flyPower);
    }
    
    private void rampFlywheelDown(){
        if(flyPower > 0 && rampTimer.time() > 115){
            flyPower -= 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
        }
        if(flyPower == 0) flywheelOn = false;
        flywheelL.setPower(flyPower);
        flywheelR.setPower(flyPower);
    }

    private void checkGamepad1(){
        if(gamepad1.dpad_up && !toggleStates[1]){
            if(movingStates[0]){
                intake.setPower(0);
                movingStates[0] = false;
            }
            else {
                movingStates[0] = true;
                intake.setPower(0.85);
            }
            toggleStates[1] = true;
        }
        else if(gamepad1.dpad_down && !toggleStates[1]){
            if(movingStates[0]){
                intake.setPower(0);
                movingStates[0] = false;
            }
            else {
                movingStates[0] = true;
                intake.setPower(-0.85);
            }
            toggleStates[1] = true;
        }
        else if(!gamepad1.dpad_down && !gamepad1.dpad_up) toggleStates[1] = false;
    }

    private void checkGamepad2(){
        if(gamepad2.dpad_up && !toggleStates[2]){
            if(movingStates[1]){
                belt.setPower(0);
                movingStates[1] = false;
            }
            else {
                movingStates[1] = true;
                belt.setPower(0.85);
            }
            toggleStates[2] = true;
        }
        else if(gamepad2.dpad_down && !toggleStates[2]){
            if(movingStates[1]){
                belt.setPower(0);
                movingStates[1] = false;
            }
            else {
                movingStates[1] = true;
                belt.setPower(-0.85);
            }
            toggleStates[2] = true;
        }
        else if(!gamepad2.dpad_down && !gamepad2.dpad_up) toggleStates[2] = false;

        if(gamepad2.y && !toggleStates[0]){
            if(flywheelOn) rampFlywheelDown();
            else rampFlywheelUp();
            toggleStates[0] = true;
        } else if(!gamepad2.y) toggleStates[0] = false;

        if(gamepad2.left_bumper && !toggleStates[3]){
            pushL.setPosition(pushL.getPosition() == 0.04 ? 0.9 : 0.04);
            toggleStates[3] = true;
        } else if(!gamepad2.left_bumper) toggleStates[3] = false;

        if(gamepad2.right_bumper && !toggleStates[4]){
            pushR.setPosition(pushR.getPosition() == 0.04 ? 0.9 : 0.04);
            toggleStates[4] = true;
        } else if(!gamepad2.right_bumper) toggleStates[4] = false;
    }

    @Override
    public void loop() {
        left.setPower(controllerYValues(gamepad1)[0]);
        right.setPower(controllerYValues(gamepad1)[1]);

        checkGamepad1();
        checkGamepad2();
    }


}
