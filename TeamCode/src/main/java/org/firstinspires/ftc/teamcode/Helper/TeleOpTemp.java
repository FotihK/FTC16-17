package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HP on 12/30/2016.
 */

public abstract class TeleOpTemp extends OpMode {
    public float right_trigger;
    private DcMotor intake, belt;
    protected DriveTrain driveTrain;
    private DriveTrain flywheel;
    private Servo pushL, pushR;
    protected LightSensor light_beacon, light_ground;
    private boolean[] toggleStates = new boolean[5];    //0th is intake, 1st is belt, 2nd is pushL, 3rd is pushR, 4th is sensor info
    private boolean[] movingStates = new boolean[2];    //0th is intake, 1st is belt
    protected boolean[] isOn = new boolean[4];            //0th is flywheel, 1st is intake, 2nd is belt, 3rd is sensor info
    private double[] servoStartPositions = {0.97, 0.00};    //Left, Right
    private double[] servoEndPositions = {0.55, 0.35};      //Left, Right
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
        if (flyPower <= 1.00 && rampTimer.time() > 350) {
            flyPower += 0.25;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        }
        if (flyPower >= 1.00) {
            flyPower = 1.00;
            isOn[0] = true;
        }
        flywheel.setPower(flyPower);
    }

    private void rampFlywheelDown() {
        if (flyPower > 0 && rampTimer.time() > 450) {
            flyPower -= 0.25;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        } else if (flyPower <= 0) {
            flyPower = 0;
            isOn[0] = false;
        }
        flywheel.setPower(flyPower);
    }

    private void rampFlywheelUpSlow() {
        if (flyPower <= 1.00 && rampTimer.time() > 350) {
            flyPower += 0.125;
            flywheel.setPower(flyPower);
            rampTimer.reset();
        }
        if (flyPower >= 1.00) {
            flyPower = 1.00;
            isOn[0] = true;
        }
        flywheel.setPower(flyPower);
    }

    private void rampFlywheelDownSlow() {
        if (flyPower > 0 && rampTimer.time() > 450) {
            flyPower -= 0.125;
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
                intake.setPower(1.00);
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
                intake.setPower(-1.00);
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
                belt.setPower(1.00);
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
                belt.setPower(-1.00);
                isOn[2] = true;
            }
            toggleStates[1] = true;
        } else if (!gamepad2.dpad_down && !gamepad2.dpad_up) toggleStates[1] = false;

        if (gamepad2.y) {       //y is 3
            rampFlywheelUp();
        }

        if (gamepad2.a) {       //a is 2
            rampFlywheelDown();
        }

        if (gamepad2.b) {       //b is 4
            rampFlywheelUpSlow();
        }

        if (gamepad2.x) {       //x is 1
            rampFlywheelDownSlow();
        }
    }

    public abstract void telemetry();

    @Override
    public abstract void loop();

}
