package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Siva on 11/9/2016.
 */
@Autonomous(name="MotorServoTests", group="Tests")
public class MotorServoTests extends LinearOpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor /*lift,*/ intake, elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip, /*latch,*/ load;
    private boolean[] toggleStates = new boolean[3]; //ind 0 is flywheel, ind 1 is flip servo, ind 2 is load servo
    private double flyPower = 0;
    private double flipPos = 0, loadPos = 0;
    private LightSensor light_ground, light_beacon;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        //lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");
        flip = hardwareMap.servo.get("flip");
        //latch = hardwareMap.servo.get("latch");
        load = hardwareMap.servo.get("load");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_beacon = hardwareMap.lightSensor.get("light_beacon");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);
        //latch.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(flipPos);
        //latch.setPosition(0);
        load.setPosition(loadPos);

        light_ground.enableLed(true);

        waitForStart();
        /*while(opModeIsActive()) {
            telemetry.addData("Light Val:", light_ground.getLightDetected());
            telemetry.addData("Light Val (raw): ", light_ground.getRawLightDetected());
            telemetry.update();
        }*/

        while(light_ground.getRawLightDetected() < 1.8){
            fL.setPower(-0.7);
            bL.setPower(-0.7);
            bR.setPower(-0.7);
            fR.setPower(-0.7);
        }

        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);

        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
