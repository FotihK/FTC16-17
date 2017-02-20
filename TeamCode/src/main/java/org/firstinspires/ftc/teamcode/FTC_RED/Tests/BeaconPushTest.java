package org.firstinspires.ftc.teamcode.FTC_RED.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.DriveTrain;

import java.util.ArrayList;

/**
 * Created by HP on 9/29/2016.
 */

@Autonomous(name="BeaconPushTest",group = "Tests")
public class BeaconPushTest extends LinearOpMode {
    private DriveTrain driveTrain;
    private Servo pushL, pushR;
    private ColorSensor color;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final double[] servoStartPositions = {0.94, 0.00};    //Left, Right
    private final double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    private final double maxFly = 0.4, turnPow = 0.42, regularDrive = 0.6, preciseDrive = 0.42, LINE_THRESHOLD = 1.8;
    protected int alliance = 0;                                     //Red is 1, Blue is -1

    /**
     * Initialization:
     * Contains hardware declarations, hardware settings, and variable initialization
     */
    protected void initialize() {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("right"), hardwareMap.dcMotor.get("left")); //Servos are front of bot for auto
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        color = hardwareMap.colorSensor.get("color");


        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        driveTrain.setDirection("left", DcMotorSimple.Direction.FORWARD);
        driveTrain.setDirection("right", DcMotorSimple.Direction.REVERSE);


        color.enableLed(false);
        pushL.setPosition(servoEndPositions[0]);
        pushR.setPosition(servoEndPositions[1]);

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
                        if (color.red() >= 3) checks.add(true);
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

        sleep(1000);

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
        driveTrain.setPower(preciseDrive - 0.2);
        telemetry.addLine("Pushing beacon");
        telemetry.update();
        while ((color.red() < 3 && color.blue() < 5) && opModeIsActive()) sleep(1);
        driveTrain.stop();
        pressButton();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.setAutoClear(true);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                alliance = 1;
            } else if(gamepad1.b) {
                alliance = -1;
            }
            if(gamepad1.y) pushBeacon();
            if(gamepad1.x) {
                pushL.setPosition(servoEndPositions[0]);
                pushR.setPosition(servoEndPositions[1]);
            }

            telemetry.addData("Color: ", alliance == 1 ? "Red" : "Blue");
            telemetry.update();
        }

    }
}
