package org.firstinspires.ftc.teamcode.FTC_RED.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by HP on 12/30/2016.
 */

@SuppressWarnings("WeakerAccess")
public abstract class AutonomousTemp extends LinearOpMode {
    private DcMotor intake, belt;
    protected DriveTrain driveTrain;
    private DriveTrain flywheel;
    protected Servo pushL, pushR;
    protected LightSensor light_beacon, light_ground;
    protected double flyPower = 0;
    protected final double[] servoStartPositions = {0.97, 0.00};    //Left, Right
    protected final double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    protected final double BEACON_THRESHOLD = 1.875;            //Less than for blue, greater than for red
    protected final double LINE_THRESHOLD = 1.8;
    protected int alliance = 0;                                     //Red is 1, Blue is -1


    protected void initialize() {
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
        driveTrain.setDirection("left", DcMotorSimple.Direction.REVERSE);
        driveTrain.setDirection("right", DcMotorSimple.Direction.FORWARD);

        flywheel.setMode("both", DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        light_beacon = hardwareMap.lightSensor.get("light_beacon");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_ground.enableLed(true);
        light_beacon.enableLed(false);

        pushL.setPosition(servoStartPositions[0]);
        pushR.setPosition(servoStartPositions[1]);
    }


    private void rampFlywheelUp() throws InterruptedException {
        while (flyPower < 1) {
            flyPower = Range.clip(flyPower + 0.25, 0, 1);
            flywheel.setPower(flyPower);
            sleep(450);
        }
    }

    private void rampFlywheelDown() throws InterruptedException {
        while (flyPower > 0) {
            flyPower = Range.clip(flyPower - 0.25, 0, 1);
            flywheel.setPower(flyPower);
            sleep(550);
        }
    }

    protected void autoShoot() throws InterruptedException {
        rampFlywheelUp();
        sleep(150);
        belt.setPower(1);
        sleep(1000);
        belt.setPower(0);
        rampFlywheelDown();
    }

    protected void moveToLine() {
        driveTrain.setPower(0.5);
        while (light_ground.getLightDetected() < LINE_THRESHOLD) {
            idle();
        }
        driveTrain.stop();
        sleep(500);
        driveTrain.setPower(-0.1);
        sleep(150);
        driveTrain.stop();
    }

    private void forward() {
        driveTrain.setPower(-0.3);
        sleep(350);
        driveTrain.stop();
        sleep(150);
    }

    protected void backward() {
        driveTrain.setPower(0.15);
        sleep(100);
        driveTrain.stop();
        sleep(150);
    }


    protected void pressBlue() {
        if (light_beacon.getRawLightDetected() <= BEACON_THRESHOLD) {
            forward();
            backward();
        }
    }

    protected void pressRed() {
        if (light_beacon.getRawLightDetected() > BEACON_THRESHOLD) {
            forward();
            backward();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();                                   //initializes, waits for start
        waitForStart();
        pushL.setPosition(servoEndPositions[0]);        //puts servos down
        pushR.setPosition(servoEndPositions[1]);

        sleep(5000);
        moveToLine();                                   //Waits 5 seconds, moves to first beacon line
        sleep(250);

        driveTrain.turn("left", 0.5 * alliance);                   //Turns towards beacon
        sleep(500);                 //TODO towards beacon
        driveTrain.stop();
        sleep(150);

        driveTrain.setPower(0.5);                       //Ram into beacon to activate
        sleep(1000);                //TODO minimize
        driveTrain.stop();
        sleep(150);

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        if (alliance == 1) pressBlue();
        else pressRed();

        driveTrain.setPower(-0.5);                      //Backs away from the beacon to get ready to shoot preloads
        sleep(1000);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(150);

        autoShoot();                                    //Automatically shoots, see AutonomouTemp

        driveTrain.setPower(0.5);
        sleep(1000);
        driveTrain.stop();
        sleep(150);

        driveTrain.turn("right", 0.5 * alliance);                   //Turns towards next beacon line
        sleep(500);                 //TODO ~90 deg towards next beacon line
        driveTrain.stop();
        sleep(150);

        moveToLine();                                   //Moves to next beacon line
        sleep(250);

        driveTrain.turn("left", 0.5 * alliance);                   //Turns towards next beacon
        sleep(500);                 //TODO ~90 deg towards next beacon
        driveTrain.stop();
        sleep(150);

        driveTrain.setPower(0.5);                       //Ram into beacon to activate
        sleep(1000);                //TODO minimize
        driveTrain.stop();
        sleep(150);

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        if (alliance == 1) pressBlue();
        else pressRed();

        driveTrain.setPower(-0.5);                      //Backs away from the beacon to turn towards ramp
        sleep(1000);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(150);

        driveTrain.turn("left", 0.5 * alliance);                   //Turns towards front of ramp
        sleep(500);                 //TODO ~90 deg towards front of ramp
        driveTrain.stop();
        pushL.setPosition(servoStartPositions[0]);        //puts servos up
        pushR.setPosition(servoStartPositions[1]);
        sleep(150);

        driveTrain.setPower(0.5);                      //Moves towards front of ramp
        sleep(2500);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(150);

        driveTrain.turn("right", 0.5 * alliance);                   //Turns towards ramp
        sleep(500);                 //TODO ~45 deg towards ramp
        driveTrain.stop();
        sleep(150);

        sleep(500);

        driveTrain.setPower(0.7);       //Climb up onto ramp and keep climbing
        while (opModeIsActive()) {
            sleep(1);
        }
    }
}

