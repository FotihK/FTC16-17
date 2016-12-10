package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by fotih on 12/9/2016.
 */

public class AutonomousRED extends LinearOpMode {
    private DcMotor left, right;
    private DcMotor intake, belt, flywheelL, flywheelR;
    private Servo pushL, pushR;
    private double flyPower = 0;

    public void initialize(){
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

    private void rampFlywheelUp() throws InterruptedException{
        while(flyPower < 0.95){
            flyPower += 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
            sleep(100);
        }
    }

    private void rampFlywheelDown() throws InterruptedException{
        while(flyPower > 0){
            flyPower -= 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
            sleep(100);
        }
    }

    private void autoShoot() throws InterruptedException{
        rampFlywheelUp();
        sleep(250);
        intake.setPower(0.85);
        belt.setPower(0.85);
        sleep(350);
        intake.setPower(0);
        belt.setPower(0);
        sleep(1000);
        intake.setPower(0.85);
        belt.setPower(0.85);
        sleep(350);
        intake.setPower(0);
        belt.setPower(0);
        rampFlywheelDown();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        sleep(7500);



    }
}
