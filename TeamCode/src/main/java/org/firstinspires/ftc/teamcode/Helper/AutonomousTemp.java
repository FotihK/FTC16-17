package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by HP on 12/30/2016.
 */

public abstract class AutonomousTemp extends LinearOpMode {
    private DcMotor intake, belt, flywheelL, flywheelR;
    protected DriveTrain driveTrain;
    protected Servo pushL, pushR;
    protected LightSensor light_beacon, light_ground;
    private double flyPower = 0;
    
    protected void initialize(){
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"), gamepad1);
        intake = hardwareMap.dcMotor.get("intake");
        belt = hardwareMap.dcMotor.get("belt");
        flywheelL = hardwareMap.dcMotor.get("flywheelL");
        flywheelR = hardwareMap.dcMotor.get("flywheelR");
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);

        flywheelL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        light_beacon = hardwareMap.lightSensor.get("light_beacon");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_ground.enableLed(true);
        light_beacon.enableLed(false);

        pushL.setPosition(0.9);
        pushR.setPosition(0.04);
    }

    private void rampFlywheelUp() throws InterruptedException{
        while(flyPower < 0.95){
            if((flyPower + 0.05) > 0.95) {
                flyPower = 0.95;
            } else flyPower += 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
            sleep(400);
        }
    }

    private void rampFlywheelDown() throws InterruptedException{
        while(flyPower > 0){
            if((flyPower - 0.05) <= 0) {
                flyPower = 0;
            } else flyPower -= 0.05;
            flywheelL.setPower(flyPower);
            flywheelR.setPower(flyPower);
            sleep(850);
        }
    }

    protected void autoShoot() throws InterruptedException{
        rampFlywheelUp();
        sleep(150);
        intake.setPower(0.85);
        belt.setPower(0.85);
        sleep(250);
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
    
    public abstract void telemetry();

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
