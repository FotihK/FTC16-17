package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.DriveTrain;


/**
 * Created by HP on 11/30/2016.
 */
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "TeleOp", group = "OpModes")
public class MainTeleOp extends OpMode {
    private DcMotor intake, belt;
    protected DriveTrain driveTrain;
    private DriveTrain flywheel;
    private Servo pushL, pushR;
    protected LightSensor light_beacon, light_ground;
    private boolean[] toggleStates = new boolean[5];    //0th is intake, 1st is belt, 2nd is pushL, 3rd is pushR, 4th is sensor info
    private boolean[] movingStates = new boolean[2];    //0th is intake, 1st is belt
    protected boolean[] isOn = new boolean[4];            //0th is flywheel, 1st is intake, 2nd is belt, 3rd is sensor info
    private double[] servoStartPositions = {0.95, 0.15};    //Left, Right
    private double[] servoEndPositions = {0.53, 0.57};      //Left, Right
    protected double flyPower = 0;
    private ElapsedTime rampTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {                                //Initializes hardware with directions and positions
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
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

        flywheel.setMode("both", DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        light_beacon = hardwareMap.lightSensor.get("light_beacon");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_ground.enableLed(false);
        light_beacon.enableLed(false);

        pushL.setPosition(servoStartPositions[0]);
        pushR.setPosition(servoStartPositions[1]);

    }

    private void rampFlywheelUp() {
        if (flyPower <= 0.95 && rampTimer.time() > 350) {
            flyPower += 0.065;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        }
        if (flyPower >= 0.95) {
            flyPower = 0.95;
            isOn[0] = true;
        }
        flywheel.setPower(flyPower);
    }

    private void rampFlywheelDown() {
        if (flyPower > 0 && rampTimer.time() > 450) {
            flyPower -= 0.05;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        } else if (flyPower <= 0) {
            flyPower = 0;
            isOn[0] = false;
        }
        flywheel.setPower(flyPower);
    }

    protected void checkGamepad1() {                       //Buttons for gamepad 1
        if (gamepad1.dpad_up && !toggleStates[0]) {       //Up toggle for intake
            if (movingStates[0]) {
                intake.setPower(0);
                movingStates[0] = false;
                isOn[1] = false;
            } else {
                movingStates[0] = true;
                intake.setPower(0.85);
                isOn[1] = true;
            }
            toggleStates[0] = true;
        } else if (gamepad1.dpad_down && !toggleStates[0]) {    //Down toggle for intake
            if (movingStates[0]) {
                intake.setPower(0);
                movingStates[0] = false;
                isOn[1] = false;
            } else {
                movingStates[0] = true;
                intake.setPower(-0.85);
                isOn[1] = true;
            }
            toggleStates[0] = true;
        } else if (!gamepad1.dpad_down && !gamepad1.dpad_up) toggleStates[0] = false;

        if (gamepad1.left_bumper && !toggleStates[2]) {   //Toggle for pushL
            pushL.setPosition(pushL.getPosition() == servoStartPositions[0] ? servoEndPositions[0] : servoStartPositions[0]);
            toggleStates[2] = true;
        } else if (!gamepad1.left_bumper) toggleStates[2] = false;

        if (gamepad1.right_bumper && !toggleStates[3]) {  //Toggle for pushR
            pushR.setPosition(pushR.getPosition() == servoStartPositions[1] ? servoEndPositions[1] : servoStartPositions[1]);
            toggleStates[3] = true;
        } else if (!gamepad1.right_bumper) toggleStates[3] = false;

        if (gamepad1.start && !toggleStates[4]) {
            isOn[3] = !isOn[3];
            light_ground.enableLed(isOn[3]);
            toggleStates[4] = true;
        } else if (!gamepad1.start) toggleStates[4] = false;

    }

    protected void checkGamepad2() {                   //Buttons for gamepad 2
        if (gamepad2.dpad_up && !toggleStates[1]) {   //Up toggle for belt
            if (movingStates[1]) {
                belt.setPower(0);
                movingStates[1] = false;
                isOn[2] = false;
            } else {
                movingStates[1] = true;
                belt.setPower(0.85);
                isOn[2] = true;
            }
            toggleStates[1] = true;
        } else if (gamepad2.dpad_down && !toggleStates[1]) {    //Down toggle for belt
            if (movingStates[1]) {
                belt.setPower(0);
                movingStates[1] = false;
                isOn[2] = false;
            } else {
                movingStates[1] = true;
                belt.setPower(-0.85);
                isOn[2] = true;
            }
            toggleStates[1] = true;
        } else if (!gamepad2.dpad_down && !gamepad2.dpad_up) toggleStates[1] = false;

        if (gamepad2.y) {
            rampFlywheelUp();
        }

        if (gamepad2.b) {
            rampFlywheelDown();
        }

    }

    protected void telemetry() {
        telemetry.addData("Flywheel is ", (isOn[0] ? "on. " : "off."));
        telemetry.addData("Intake is ", (isOn[1] ? "on. " : "off."));
        telemetry.addData("Belt is ", (isOn[2] ? "on. " : "off."));
        telemetry.addData("Left motor power: ", driveTrain.getGamepadYValues(gamepad1)[0]);
        telemetry.addData("Right motor power: ", driveTrain.getGamepadYValues(gamepad1)[1]);
        telemetry.addData("Flywheel power :", flyPower);
        if (isOn[3]) {
            telemetry.addData("Ground Sensor Raw/Detected: ", light_ground.getRawLightDetected() + "/" + light_ground.getLightDetected());
            telemetry.addData("Beacon Sensor Raw/Detected: ", light_beacon.getRawLightDetected() + "/" + light_beacon.getLightDetected());
        }
    }

    @Override
    public void loop() {
        driveTrain.tankDrive(gamepad1);
        checkGamepad1();
        checkGamepad2();
        telemetry();
    }


}
