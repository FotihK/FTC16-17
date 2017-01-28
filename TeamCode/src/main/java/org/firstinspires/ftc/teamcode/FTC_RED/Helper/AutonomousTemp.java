package org.firstinspires.ftc.teamcode.FTC_RED.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/**
 * Created by HP on 12/30/2016.
 */

public abstract class AutonomousTemp extends LinearOpMode {
    private DcMotor intake, belt;
    private DriveTrain driveTrain;
    private DriveTrain flywheel;
    private Servo pushL, pushR;
    private LightSensor light_beacon, light_ground;
    private double flyPower = 0;
    private final double[] servoStartPositions = {0.97, 0.00};    //Left, Right
    private final double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    private final double BEACON_THRESHOLD = 1.875;            //Less than for blue, greater than for red
    private final double LINE_THRESHOLD = 1.8;
    protected int alliance = 0;                                     //Red is 1, Blue is -1
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private GyroSensor gyro;
    private double offset, heading;
    private long lastTime;
    private Thread gyroThread;

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

        gyro = hardwareMap.gyroSensor.get("gyro");

        offset = 0;
        genOffset();
        lastTime = System.currentTimeMillis();
        heading = 0;

        gyroThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    integrateGyro();
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });

        gyroThread.start();

    }

    private void integrateGyro() {
        long currTime = System.currentTimeMillis();
        if (Math.abs((gyro.getRotationFraction() - offset)) * ((currTime - lastTime)) > 0.025)
            heading += ((gyro.getRotationFraction() - offset)) * ((currTime - lastTime));
        lastTime = currTime;
        if (heading >= 360.5) heading -= 360;
        if (heading < -0.5) heading += 360;
    }

    private void turn(double power, double angle) {
        double newHeading = heading + angle;
        boolean wrap = false;
        if (newHeading >= 360.5) {
            newHeading -= 360;
            wrap = true;
        }
        if (newHeading < -0.5) {
            newHeading += 360;
            wrap = true;
        }

        if (wrap) {
            if ((heading + angle) > heading) {
                driveTrain.turn("right", power);
                while (heading > newHeading && opModeIsActive()) {
                    idle();
                }
            } else {
                driveTrain.turn("left", power);
                while (heading < newHeading && opModeIsActive()) {
                    idle();
                }
            }
        } else {
            if ((heading + angle) > heading) {
                driveTrain.turn("right", power);
                while (heading < newHeading && opModeIsActive()) {
                    idle();
                }
            } else {
                driveTrain.turn("left", power);
                while (heading > newHeading && opModeIsActive()) {
                    idle();
                }
            }
        }
        driveTrain.stop();


    }

    private void genOffset(){
        int sum = 0;
        for(int i = 0; (i < 50) && (opModeIsActive()); i++){
            sum += gyro.getRotationFraction();
            sleep(15);
        }
        offset = sum / 50;
    }

    private void rampFlywheelUp() throws InterruptedException {
        while (flyPower < 1 && opModeIsActive()) {
            flyPower = Range.clip(flyPower + 0.25, 0, 1);
            flywheel.setPower(flyPower);
            sleep(350);
        }
    }

    private void rampFlywheelDown() throws InterruptedException {
        while (flyPower > 0 && opModeIsActive()) {
            flyPower = Range.clip(flyPower - 0.15, 0, 1);
            flywheel.setPower(flyPower);
            sleep(450);
        }
    }

    private void autoShoot() throws InterruptedException {
        rampFlywheelUp();
        sleep(75);
        belt.setPower(1);
        sleep(1000);
        belt.setPower(0);
        rampFlywheelDown();
    }

    private void moveToLine() {
        driveTrain.setPower(0.7);
        while (light_ground.getLightDetected() < LINE_THRESHOLD) {
            idle();
        }
        sleep(150);
        driveTrain.stop();
    }

    private void forward() {
        driveTrain.setPower(0.4);
        sleep(100);
        driveTrain.stop();
        sleep(75);
    }

    private void backward() {
        driveTrain.setPower(-0.2);
        sleep(75);
        driveTrain.stop();
        sleep(75);
    }

    private void pressBlue() {
        ArrayList<Boolean> checks = new ArrayList<>();
        timer.reset();
        while(timer.milliseconds() <= 1001 && opModeIsActive()){
            telemetry.addData("Raw: ", light_beacon.getRawLightDetected());
            telemetry.addData("Val: ", light_beacon.getLightDetected());
            if((timer.milliseconds()%100) >= 0 && (timer.milliseconds()%100) <= 6 ){
                if(light_beacon.getRawLightDetected() <= BEACON_THRESHOLD) checks.add(true);
            }
        }

        int count = 0;

        for(boolean check : checks){
            if(check) count++;
        }

        if (count >= (int)((checks.size() * 0.75) + 0.5) ) {
            forward();
            backward();
        }
    }

    private void pressRed() {
        ArrayList<Boolean> checks = new ArrayList<>();
        timer.reset();
        while(timer.milliseconds() <= 1001 && opModeIsActive()){
            telemetry.addData("Raw: ", light_beacon.getRawLightDetected());
            telemetry.addData("Val: ", light_beacon.getLightDetected());
            if((timer.milliseconds()%100) >= 0 && (timer.milliseconds()%100) <= 6 ){
                if(light_beacon.getRawLightDetected() > BEACON_THRESHOLD) checks.add(true);
            }
        }

        int count = 0;

        for(boolean check : checks){
            if(check) count++;
        }

        if (count >= (int)((checks.size() * 0.75) + 0.5)) {
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

        driveTrain.setPower(0.5);
        sleep(150);
        driveTrain.stop();
        turn(0.5, -45 * alliance);

        sleep(75);
        moveToLine();                                   //Waits 5 seconds, moves to first beacon line
        sleep(75);

        //driveTrain.turn("left", 0.5 * alliance);                   //Turns towards beacon
        //while (light_ground.getLightDetected() < LINE_THRESHOLD) idle();
        turn(0.5, -45 * alliance);
        driveTrain.stop();
        sleep(75);
        genOffset();


        driveTrain.setPower(0.6);                       //Ram into beacon to activate
        sleep(500);                //TODO minimize
        driveTrain.stop();
        sleep(75);

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        if (alliance == 1) pressBlue();
        else pressRed();

        driveTrain.setPower(-0.5);                      //Backs away from the beacon to get ready to shoot preloads
        sleep(1000);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(75);

        autoShoot();                                    //Automatically shoots
        sleep(75);

        driveTrain.setPower(0.5);
        sleep(900);
        driveTrain.stop();
        sleep(75);

        turn(0.5, 90 * alliance);
        driveTrain.stop();
        sleep(75);

        moveToLine();                                   //Moves to next beacon line
        sleep(75);

        turn(0.5, -90 * alliance);
        driveTrain.stop();
        sleep(75);

        driveTrain.setPower(0.6);                       //Ram into beacon to activate
        sleep(500);                //TODO minimize
        driveTrain.stop();
        sleep(75);

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        if (alliance == 1) pressBlue();
        else pressRed();

        driveTrain.setPower(-0.5);                      //Backs away from the beacon to turn towards ramp
        sleep(1000);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(75);

        turn(0.5, -90 * alliance);
        driveTrain.stop();
        sleep(75);

        pushL.setPosition(servoStartPositions[0]);        //puts servos up
        pushR.setPosition(servoStartPositions[1]);

        driveTrain.setPower(0.5);                      //Moves towards front of ramp
        sleep(2500);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(75);

        turn(0.5, 45 * alliance);
        driveTrain.stop();
        sleep(75);

        driveTrain.setPower(0.7);       //Climb up onto ramp and keep climbing
        while (opModeIsActive()) {
            idle();
        }

    }
}

