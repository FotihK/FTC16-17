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
    private boolean[] movingStates = new boolean[2];    //0th is intake, 1st is belt
    private boolean[] isOn = new boolean[3];            //0th is flywheel, 1st is intake, 2nd is belt
    private double flyPower = 0;
    private boolean flywheelOn = false;
    private ElapsedTime rampTimer = new ElapsedTime();

    @Override
    public void init() {                                //Initializes hardware with directions and positions
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

    private double[] controllerYValues(Gamepad gamepad) {   // Returns controller values for passed gamepad
        double[] returnVals = new double[2];                // index 0 is left, index 1 is right
        returnVals[0] = Math.abs(gamepad.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        returnVals[1] = Math.abs(gamepad.right_stick_y) > 0.05 ? gamepad1.right_stick_y : 0;

        return returnVals;
    }

    private void rampFlywheelUp(){                          //Ramp flywheel up, ΔP = 0.05, Δt = 115ms
        if(flyPower <= 0.95 && rampTimer.time() > 115) {
            flyPower += 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
        }
        if(flyPower == 0.95) {
            flywheelOn = true;
            isOn[0] = true;
        }
        flywheelL.setPower(flyPower);
        flywheelR.setPower(flyPower);
    }

    private void rampFlywheelDown(){                        //Ramp flywheel down, ΔP = -0.05, Δt = 115ms
        if(flyPower > 0 && rampTimer.time() > 115){
            flyPower -= 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
        }
        if(flyPower == 0) {
            flywheelOn = false;
            isOn[0] = false;
        }
        flywheelL.setPower(flyPower);
        flywheelR.setPower(flyPower);

    }

    private void checkGamepad1(){                       //Buttons for gamepad 1
        if(gamepad1.dpad_up && !toggleStates[1]){       //Up toggle for intake
            if(movingStates[0]){
                intake.setPower(0);
                movingStates[0] = false;
                isOn[1] = false;
            }
            else {
                movingStates[0] = true;
                intake.setPower(0.85);
                isOn[1] = true;
            }
            toggleStates[1] = true;
        }
        else if(gamepad1.dpad_down && !toggleStates[1]){    //Down toggle for intake
            if(movingStates[0]){
                intake.setPower(0);
                movingStates[0] = false;
                isOn[1] = false;
            }
            else {
                movingStates[0] = true;
                intake.setPower(-0.85);
                isOn[1] = true;
            }
            toggleStates[1] = true;
        }
        else if(!gamepad1.dpad_down && !gamepad1.dpad_up) toggleStates[1] = false;
    }

    private void checkGamepad2(){                   //Buttons for gamepad 2
        if(gamepad2.dpad_up && !toggleStates[2]){   //Up toggle for belt
            if(movingStates[1]){
                belt.setPower(0);
                movingStates[1] = false;
                isOn[2] = false;
            }
            else {
                movingStates[1] = true;
                belt.setPower(0.85);
                isOn[2] = true;
            }
            toggleStates[2] = true;
        }
        else if(gamepad2.dpad_down && !toggleStates[2]){    //Down toggle for belt
            if(movingStates[1]){
                belt.setPower(0);
                movingStates[1] = false;
                isOn[2] = false;
            }
            else {
                movingStates[1] = true;
                belt.setPower(-0.85);
                isOn[2] = true;
            }
            toggleStates[2] = true;
        }
        else if(!gamepad2.dpad_down && !gamepad2.dpad_up) toggleStates[2] = false;

        if(gamepad2.y && !toggleStates[0]){             //Toggle for flywheel
            if(flywheelOn) rampFlywheelDown();
            else rampFlywheelUp();
            toggleStates[0] = true;
        } else if(!gamepad2.y) toggleStates[0] = false;

        if(gamepad2.left_bumper && !toggleStates[3]){   //Toggle for pushL
            pushL.setPosition(pushL.getPosition() == 0.04 ? 0.9 : 0.04);
            toggleStates[3] = true;
        } else if(!gamepad2.left_bumper) toggleStates[3] = false;

        if(gamepad2.right_bumper && !toggleStates[4]){  //Toggle for pushR
            pushR.setPosition(pushR.getPosition() == 0.04 ? 0.9 : 0.04);
            toggleStates[4] = true;
        } else if(!gamepad2.right_bumper) toggleStates[4] = false;
    }

    private void telemetry(){
        telemetry.addData("Flywheel is ", (isOn[0] ? "on. " : "off."));
        telemetry.addData("Intake is ", (isOn[1] ? "on. " : "off."));
        telemetry.addData("Belt is ", (isOn[2] ? "on. " : "off."));
    }

    @Override
    public void loop() {
        left.setPower(controllerYValues(gamepad1)[0]);
        right.setPower(controllerYValues(gamepad1)[1]);

        checkGamepad1();
        checkGamepad2();

        telemetry();
    }


}
