package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.DriveTrain;

@Autonomous(name="Straight2")
public class AutonomousSTRAIGHT2 extends LinearOpMode {
    private DcMotor intake, belt;
    private DriveTrain driveTrain, flywheel;
    private Servo pushL, pushR;
    private double flyPower = 0, offset, heading;
    private final double[] servoStartPositions = {0.94, 0.00};    //Left, Right
    private final double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    private final double maxFly = 0.4, turnPow = 0.42, regularDrive = 0.6, preciseDrive = 0.42, LINE_THRESHOLD = 1.8;
    private long lastTime = System.currentTimeMillis();
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private GyroSensor gyro;

    /**
     * Allows for the autonomous code to run while the gyro continually integrates
     */
    private Thread backThread = new Thread(new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                integrateGyro();
            }
        }
    });

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
            flyPower = Range.clip(flyPower + (maxFly / 2.0), 0, maxFly);
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
            flyPower = Range.clip(flyPower - (maxFly / 3.0), 0, maxFly);
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

    private void initialize(){
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        flywheel = new DriveTrain(hardwareMap.dcMotor.get("flywheelL"), hardwareMap.dcMotor.get("flywheelR"));
        intake = hardwareMap.dcMotor.get("intake");
        belt = hardwareMap.dcMotor.get("belt");
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        gyro = hardwareMap.gyroSensor.get("gyro");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection("left", DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection("right", DcMotorSimple.Direction.FORWARD);
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);

        flywheel.setMode("both", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pushL.setPosition(servoStartPositions[0]);
        pushR.setPosition(servoStartPositions[1]);

        lastTime = System.currentTimeMillis();
        offset = gyro.getRotationFraction();
        heading = 0;
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

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        backThread.start();

        sleep(10000);
        driveTrain.setPower(0.6);
        sleep(9715);
        driveTrain.stop();
        autoShoot();
        driveTrain.setPower(regularDrive);
        sleep(1500);
        driveTrain.stop();
    }
}
