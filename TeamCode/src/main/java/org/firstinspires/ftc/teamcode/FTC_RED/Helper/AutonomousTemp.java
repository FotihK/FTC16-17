package org.firstinspires.ftc.teamcode.FTC_RED.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Objects;

/**
 * Created by HP on 12/30/2016.
 */

//glhf

@SuppressWarnings("FieldCanBeLocal")
public class AutonomousTemp extends LinearOpMode {
    private DcMotor intake, belt;
    private DriveTrain driveTrain, flywheel;
    private Servo pushL, pushR;
    private LightSensor light_ground;
    private ColorSensor color;
    private UltrasonicSensor sonar;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private GyroSensor gyro;

    private double flyPower = 0, offset = 0, heading = 0, distance = 0;
    private final double[] servoStartPositions = {0.94, 0.00};    //Left, Right
    private final double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    private final double maxFly = 0.5, turnPow = 0.42, regularDrive = 0.6, preciseDrive = 0.39, LINE_THRESHOLD = 1.8;
    ;
    protected int alliance = 0;                                     //Red is 1, Blue is -1
    private long lastTime = System.currentTimeMillis();

    /**
     * Allows for the autonomous code to run while the gyro continually integrates
     */
    private Thread backThread = new Thread(new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                integrateGyro();
                if (Math.abs(distance - sonar.getUltrasonicLevel()) < 9 || Math.abs(distance - sonar.getUltrasonicLevel()) > 70)
                    distance = sonar.getUltrasonicLevel();
                telemetry.addData("Dist:", distance);
                telemetry.update();
            }
        }
    });

    /**
     * Initialization:
     * Contains hardware declarations, hardware settings, and variable initialization
     */
    protected void initialize() {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("right"), hardwareMap.dcMotor.get("left")); //Servos are front of bot for auto
        flywheel = new DriveTrain(hardwareMap.dcMotor.get("flywheelL"), hardwareMap.dcMotor.get("flywheelR"));
        intake = hardwareMap.dcMotor.get("intake");
        belt = hardwareMap.dcMotor.get("belt");
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        color = hardwareMap.colorSensor.get("color");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        gyro = hardwareMap.gyroSensor.get("gyro");
        sonar = hardwareMap.ultrasonicSensor.get("sonar");


        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection("left", DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection("right", DcMotorSimple.Direction.FORWARD);
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        driveTrain.setDirection("left", DcMotorSimple.Direction.FORWARD);
        driveTrain.setDirection("right", DcMotorSimple.Direction.REVERSE);


        flywheel.setMode("both", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        light_ground.enableLed(true);
        color.enableLed(false);
        pushL.setPosition(servoStartPositions[0]);
        pushR.setPosition(servoStartPositions[1]);


        lastTime = System.currentTimeMillis();
        offset = gyro.getRotationFraction();
        heading = 0;
        distance = 2;
        telemetry.setAutoClear(true);
        telemetry.addData("Offset: ", offset);
        telemetry.addData("Dist: ", distance);
        telemetry.update();
    }

    /**
     * Gyroscope returns angular velocity, which is the derivative of angle, in thousandths of a degree per second
     * In order to have an angle for use, this returned value must be integrated
     * This method multiplies the angular velocity by the change in time to get the angle of movement that occured in that time period
     * This is then added to the overall angle, stored in the variable "heading"
     * This method of integration is called the rectangular approximation method (RAM), and the midpoint version is being used in this instance
     * Slightly inaccurate, as is the case with all forms of numerical integration, however it is good enough for our use
     *
     * @see #heading
     * @see #backThread
     */
    private void integrateGyro() {
        long currTime = System.currentTimeMillis();
        if (Math.abs((gyro.getRotationFraction() - offset)) * ((currTime - lastTime)) > 0.015)
            heading += ((gyro.getRotationFraction() - offset)) * ((currTime - lastTime));
        lastTime = currTime;
    }

    /**
     * Using gyroscope integration, turns the specified number of degrees at the specified power
     * Slightly inaccurate due to numerical integration
     *
     * @param power   - The power at which to turn
     * @param degrees - The number of degrees to turn
     */
    private void turn(double power, double degrees) {
        double newHeading = heading + degrees;
        if ((heading + degrees) > heading) {
            driveTrain.turn("right", power);
            while (heading < newHeading && opModeIsActive()) sleep(1);
        } else {
            driveTrain.turn("left", power);
            while (heading > newHeading && opModeIsActive()) sleep(1);
        }
        driveTrain.stop();
    }

    /**
     * Slowly increases the power of the flywheel by increments of 0.25 at 250ms intervals
     * This is to lessen the shock of the gear train suddenly starting to the motors
     */
    private void rampFlywheelUp() {
        while (flyPower < maxFly && opModeIsActive()) {
            flyPower = Range.clip(flyPower + 0.25, 0, maxFly);
            flywheel.setPower(flyPower);
            sleep(250);
        }
    }

    /**
     * Slowly decreases the power of the flywheel at increments of 0.2 at 400ms intervals
     * This is to lessen the shock of the gear train suddenly stopping to the motors
     */
    private void rampFlywheelDown() {
        while (flyPower > 0 && opModeIsActive()) {
            flyPower = Range.clip(flyPower - 0.2, 0, maxFly);
            flywheel.setPower(flyPower);
            sleep(400);
        }
    }

    /**
     * Method to automatically shoot preloaded balls
     */
    private void autoShoot() {
        rampFlywheelUp();
        belt.setPower(1);
        sleep(2750);
        belt.setPower(0);
        rampFlywheelDown();
    }

    /**
     * Method to move forward until a line is found
     */
    private void moveToLine() {
        driveTrain.setPower(preciseDrive - 0.05);
        while (light_ground.getRawLightDetected() < LINE_THRESHOLD && opModeIsActive()) sleep(1);
        sleep(100);
        driveTrain.stop();
    }

    /**
     * Method to poll color sensor while at beacon to ensure correct color is pressed
     * Pulls color sensor value every 25ms for 300ms and checks whether it is the alliance color, adding this boolean into an ArrayList
     * Afterwards, checks to see if 75% of the values in the ArrayList are true
     * If yes, the right (sensor) arm is lowered and the left arm is raised, otherwise, the opposite happens
     * Then, forward movement to push the correct button
     */
    private void pressButton() {
        telemetry.addLine("Pushing button");
        telemetry.update();
        ArrayList<Boolean> checks = new ArrayList<>();
        timer.reset();
        while (timer.milliseconds() <= 300 && opModeIsActive()) {
            if ((timer.milliseconds() % 25) >= 0 && (timer.milliseconds() % 25) <= 2) {
                switch (alliance) {
                    case 1:
                        if (color.red() >= 4) checks.add(true);
                        else checks.add(false);
                        break;
                    case -1:
                        if (color.blue() >= 5) checks.add(true);
                        else checks.add(false);
                        break;
                }
            }
        }

        int count = 0;
        for (boolean check : checks) {
            if (check) count++;
        }

        if (count >= (checks.size() * 3) / 4) {
            pushL.setPosition(servoStartPositions[0]);
            pushR.setPosition(servoEndPositions[1]);
        } else {
            pushL.setPosition(servoEndPositions[0]);
            pushR.setPosition(servoStartPositions[1]);
        }

        sleep(500);

        driveTrain.setPower(regularDrive);
        telemetry.addLine("Done");
        telemetry.update();
        sleep(300);
        driveTrain.stop();
    }

    /**
     * Helper method to previous method
     * Places robot at a location where the sensor can accurately poll the beacon
     *
     * @see #pressButton()
     */
    private void pushBeacon() {
        driveTrain.setPower(preciseDrive - 0.05);
        telemetry.addLine("Pushing beacon");
        telemetry.update();
        while ((color.red() < 4 && color.blue() < 5) && opModeIsActive()) sleep(1);
        driveTrain.stop();
        pressButton();
    }

    /**
     * Method to move a specified distance using the NXT Ultrasonic Sensor
     *
     * @param dir  - Direction: forward, backward
     * @param dist - The distance to move
     */
    private void moveDistance(String dir, double dist) {               //TODO: Add second US Sensor
        if (Objects.equals(dir, "forward")) driveTrain.setPower(preciseDrive);
        else driveTrain.setPower(-preciseDrive);
        double curr = distance;
        if (Objects.equals(dir, "forward"))
            while (distance <= curr + dist && opModeIsActive()) sleep(1);
        else while (distance >= curr - dist && opModeIsActive()) sleep(1);
        driveTrain.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();                                               //Initializes, waits for start
        waitForStart();
        backThread.start();

        ///////////////START///////////////////////////////////////////
        //moveDistance("forward", 32);                                           //Moves (26) cm forward       ---move start
        driveTrain.setPower(preciseDrive);
        int t = 1500;
        sleep(t);
        turn(turnPow, 174.5);
        //autoShoot();
        sleep(1000);
        turn(turnPow, -174.5 * alliance);
        sleep(250);
        driveTrain.setPower(-preciseDrive);
        sleep(t/2);
        driveTrain.stop();
        //moveDistance("backward", 15);

        turn(turnPow, -47 * alliance);                              //Turns towards beacon line      -----turn

        moveToLine();                                               //Moves to first beacon line

        pushL.setPosition(servoEndPositions[0]);                    //Puts servos down
        pushR.setPosition(servoEndPositions[1]);

        //turn(turnPow, -45 * alliance);                              //Turns towards first beacon
        driveTrain.turn("l", -turnPow * alliance);
        while (light_ground.getRawLightDetected() < LINE_THRESHOLD && opModeIsActive()) sleep(1);
        //while ((color.red() < 4 && color.blue() < 5) && opModeIsActive()) sleep(1);
        driveTrain.stop();

        pushBeacon();                                               //Reads beacon and pushes appropriate button
        driveTrain.setPower(-0.5);
        sleep(150);
        driveTrain.stop();

        turn(turnPow, 85 * alliance);                               //Turns towards next beacon line

        moveToLine();                                               //Moves to next beacon line

//        driveTrain.setPower(-(preciseDrive - 0.05));
//        sleep(50);
//        driveTrain.stop();

        pushL.setPosition(servoEndPositions[0]);                    //Puts servos down
        pushR.setPosition(servoEndPositions[1]);

        turn(turnPow, -90 * alliance);                              //Turns towards beacon

        pushBeacon();                                               //Reads beacon and pushes appropriate button

        driveTrain.setPower(-regularDrive);                         //Backs away from the beacon to turn towards ramp
        sleep(800);                                                 //TODO: Replace with Ultrasonic Sensor
        driveTrain.stop();

        turn(turnPow, -90 * alliance);                              //Turns towards ramp

        pushL.setPosition(servoStartPositions[0]);                  //Puts servos up
        pushR.setPosition(servoStartPositions[1]);

        driveTrain.setPower(regularDrive);                          //Moves towards front of ramp
        sleep(2500);                                                //TODO: Find timing for optimal distance, possibly US sensor
        driveTrain.stop();

        turn(turnPow, 45 * alliance);                               //Turn towards ramp

        driveTrain.setPower(0.5);                                   //Climb up onto ramp and continue climbing
        while (opModeIsActive()) {
            sleep(1);
        }
    }
}

