package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.DriveTrain;

@Autonomous(name="straight")
public class AutonomousSTRAIGHT extends LinearOpMode {
    private DcMotor intake, belt;
    private DriveTrain driveTrain, flywheel;
    private Servo pushL, pushR;
    private double flyPower = 0, offset, heading;
    private final double[] servoStartPositions = {0.94, 0.00};    //Left, Right
    private final double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    private final double maxFly = 0.525;

    private void rampFlywheelUp() throws InterruptedException {
        while (flyPower < maxFly && opModeIsActive()) {
            flyPower = Range.clip(flyPower + 0.25, 0, maxFly);
            flywheel.setPower(flyPower);
            sleep(350);
        }
    }

    private void rampFlywheelDown() throws InterruptedException {
        while (flyPower > 0 && opModeIsActive()) {
            flyPower = Range.clip(flyPower - 0.15, 0, maxFly);
            flywheel.setPower(flyPower);
            sleep(450);
        }
    }

    private void autoShoot() throws InterruptedException {
        rampFlywheelUp();
        sleep(75);
        belt.setPower(1);
        sleep(2500);
        belt.setPower(0);
        rampFlywheelDown();
    }

    private void initialize(){
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("right"), hardwareMap.dcMotor.get("left")); //Servos are front of bot for auto
        flywheel = new DriveTrain(hardwareMap.dcMotor.get("flywheelL"), hardwareMap.dcMotor.get("flywheelR"));
        intake = hardwareMap.dcMotor.get("intake");
        belt = hardwareMap.dcMotor.get("belt");
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection("left", DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection("right", DcMotorSimple.Direction.FORWARD);
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        driveTrain.setDirection("left", DcMotorSimple.Direction.FORWARD);
        driveTrain.setDirection("right", DcMotorSimple.Direction.REVERSE);

        flywheel.setMode("both", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pushL.setPosition(servoStartPositions[0]);
        pushR.setPosition(servoStartPositions[1]);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        pushL.setPosition(servoEndPositions[0]);        //puts servos down
        pushR.setPosition(servoEndPositions[1]);

        sleep(10000);
        driveTrain.setPower(-0.5);
        sleep(1400);
        driveTrain.stop();

        autoShoot();
        sleep(100);
        driveTrain.setPower(-0.5);
        sleep(1400);

        driveTrain.stop();

    }
}
