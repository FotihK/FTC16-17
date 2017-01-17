package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.DriveTrain;

/**
 * Created by Siva on 11/9/2016.
 */
@Autonomous(name="RedSide-PrafulBot", group="Autonomous")
public class Auto_RedSide_PrafulBot extends LinearOpMode {
    private DcMotor fL, fR, bL, bR;
    private DriveTrain front, back;
    private DcMotor intake, elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip, load;
    private double flipPos = 0, loadPos = 0;
    private LightSensor light_ground, light_beacon;

    private void stopDrive() {        //Method to stop drive motors
        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);
        sleep(150);
    }

    public void initialize(){
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");
        flip = hardwareMap.servo.get("flip");
        load = hardwareMap.servo.get("load");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_beacon = hardwareMap.lightSensor.get("light_beacon");

        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //UNCOMMENT IF USING ANDYMARK
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //COMMENT IF USING ANDYMARK
        bL.setMaxSpeed(2800);
        bR.setMaxSpeed(2800);


        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(flipPos);
        load.setPosition(loadPos);

        light_ground.enableLed(true);
        light_beacon.enableLed(false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        fL.setPower(-0.4);      //RAMP UP FOR EASY START
        bL.setPower(-0.4);
        bR.setPower(-0.4);
        fR.setPower(-0.4);

        sleep(125);

        fL.setPower(-0.2);      //SWITCH TO ACTUAL POWER
        bL.setPower(-0.2);
        bR.setPower(-0.2);
        fR.setPower(-0.2);

        while(light_ground.getRawLightDetected() < 1.8 && opModeIsActive()){    //Runs until white beacon line is seen
            idle();
        }

        stopDrive();

        fL.setPower(-0.2);
        bL.setPower(-0.2);
        bR.setPower(-0.2);
        fR.setPower(-0.2);

        fL.setPower(-0.2);
        bL.setPower(-0.2);
        bR.setPower(-0.2);
        fR.setPower(-0.2);

        stopDrive();



    }
}
