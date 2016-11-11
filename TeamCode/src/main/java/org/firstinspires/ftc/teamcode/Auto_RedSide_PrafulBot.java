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
@Autonomous(name="RedSide-PrafulBot", group="Autonomous")
public class Auto_RedSide_PrafulBot extends LinearOpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor /*lift,*/ intake, elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip, /*latch,*/ load;
    private double flipPos = 0, loadPos = 0;
    private LightSensor light_ground, light_beacon;

    public void stopDrive(){        //Method to stop drive motors
        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);
        sleep(500);
    }

    public void readyEncoders(){    //Method to reset and initialize drive train encoders
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopEncoders(){     //Method to reset and stop using drive train encoders
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize(){
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

        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //UNCOMMENT IF USING ANDYMARK
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //COMMENT IF USING ANDYMARK
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);
        //latch.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(flipPos);
        //latch.setPosition(0);
        load.setPosition(loadPos);

        light_ground.enableLed(true);
        light_beacon.enableLed(false);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while(light_ground.getRawLightDetected() < 1.8){    //Runs until white beacon line is seen
            fL.setPower(-0.2);
            bL.setPower(-0.2);
            bR.setPower(-0.2);
            fR.setPower(-0.2);
        }

        stopDrive();

        sleep(5000);

        //readyEncoders();            //Turns towards beacon
        //bL.setTargetPosition(560); //CHANGE THIS FOR AMOUNT OF SWING TURN
        //while(bL.isBusy()) {
        stopEncoders();
            bL.setPower(-1);
            fL.setPower(-1);
            bR.setPower(1);
            fR.setPower(1);
        //}

        sleep(500);
        stopDrive();

        sleep(5000);

        readyEncoders();            //Inches towards beacon
        bL.setTargetPosition(140); //CHANGE THESE FOR AMOUNT OF MOVEMENT TOWARDS BEACON
        bR.setTargetPosition(140);
        while(bL.isBusy() || bR.isBusy()){
            fL.setPower(-0.2);
            bL.setPower(-0.2);
            bR.setPower(-0.2);
            fR.setPower(-0.2);
        }

        stopDrive();

        sleep(5000);

        if(light_beacon.getRawLightDetected() < 2.05){  //Checks beacon and extends corresponding pusher
            pushL.setPower(0.5);
            sleep(1000);
            pushL.setPower(0);
        } else {
            pushR.setPower(0.5);
            sleep(1000);
            pushR.setPower(0);
        }

        sleep(5000);

        readyEncoders();            //Inches away from beacon
        bL.setTargetPosition(280); //CHANGE THESE FOR MOVEMENT AWAY FROM BEACON OR IF MOVEMENT IS NOT NEEDED
        bR.setTargetPosition(280); //COMMENT OUT FROM "readyEncoders();" TO THE END OF THE WHILE
        while(bL.isBusy() || bR.isBusy()){
            fL.setPower(0.2);
            bL.setPower(0.2);
            bR.setPower(0.2);
            fR.setPower(0.2);
        }

        stopDrive();

        sleep(5000);
               /*                     //Shoot pre-loaded particles
        flywheel.setPower(0.95); //SET POWER TO 0.75 IF USING ANDYMARK OTHERWISE SET TO 0.95
        sleep(1500);
        load.setPosition(0);
        elevator.setPower(0.75);
        sleep(1000);
        load.setPosition(1);
        sleep(250);
        load.setPosition(0);
        sleep(1000);
        load.setPosition(1);
        sleep(250);
        load.setPosition(0);
        elevator.setPower(0);
        sleep(500);
        flywheel.setPower(0);
        */
        readyEncoders();            //Turns to move towards next beacon
        bL.setTargetPosition(560); //CHANGE THIS FOR AMOUNT OF POINT TURN (~90 DEGREES NEEDED)
        while(bL.isBusy()) {
            bL.setPower(-0.3);
            fL.setPower(-0.3);
            bR.setPower(0.3);
            fR.setPower(0.3);
        }

        stopDrive();
        stopEncoders();

        sleep(5000);

        bL.setPower(-0.3); //Moves towards beacon 2 manually to ensure sensor is not on beacon 1's line
        fL.setPower(-0.3);
        bR.setPower(-0.3);
        fR.setPower(-0.3);
        sleep(500);
        stopDrive();
        sleep(5000);

        while(light_ground.getRawLightDetected() < 1.8){    //Runs until white beacon 2 line is seen
            fL.setPower(-0.7);
            bL.setPower(-0.7);
            bR.setPower(-0.7);
            fR.setPower(-0.7);
        }

        stopDrive();

        readyEncoders();            //Turns towards beacon 2
        bL.setTargetPosition(560); //CHANGE THIS FOR AMOUNT OF POINT TURN (~90 DEGREES NEEDED)
        while(bL.isBusy()) {
            bL.setPower(0.3);
            fL.setPower(0.3);
            bR.setPower(-0.3);
            fR.setPower(-0.3);
        }

        readyEncoders();            //Inches towards beacon
        bL.setTargetPosition(280); //CHANGE THESE FOR AMOUNT OF MOVEMENT TOWARDS BEACON
        bR.setTargetPosition(280);
        while(bL.isBusy() || bR.isBusy()){
            fL.setPower(-0.2);
            bL.setPower(-0.2);
            bR.setPower(-0.2);
            fR.setPower(-0.2);
        }

        if(light_beacon.getRawLightDetected() < 2.05){  //Checks beacon and extends corresponding pusher
            pushL.setPower(0.5);
            sleep(1000);
            pushL.setPower(0);
        } else {
            pushR.setPower(0.5);
            sleep(1000);
            pushR.setPower(0);
        }

    }
}
