package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Siva on 11/9/2016.
 */
@Autonomous(name="RedSide-PrafulBot", group="Autonomous")
public class Auto_RedSide_PrafulBot extends LinearOpMode {
    private DriveTrain driveTrain;
    private DcMotor elevator, flywheel;
    private CRServo pushL, pushR;
    private Servo flip;
    private Servo load;
    private double flyPower = 0;
    private LightSensor light_ground, light_beacon;
    private final double BEACON_THRESHOLD = 1.875;            //Less than for blue, greater than for red
    private final double LINE_THRESHOLD = 1.8;
    int alliance = 1;                   //1 for red, -1 for blue

    public void initialize(){
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("fL"), hardwareMap.dcMotor.get("fR"), hardwareMap.dcMotor.get("bL"),
                hardwareMap.dcMotor.get("bR"));

        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        pushL = hardwareMap.crservo.get("pushL");
        pushR = hardwareMap.crservo.get("pushR");
        flip = hardwareMap.servo.get("flip");
        load = hardwareMap.servo.get("load");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_beacon = hardwareMap.lightSensor.get("light_beacon");

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pushL.setDirection(DcMotorSimple.Direction.FORWARD);
        pushR.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(0.5);
        load.setPosition(0);

        light_ground.enableLed(true);
        light_beacon.enableLed(false);
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
            sleep(450);
        }
    }

    private void autoShoot() throws InterruptedException{
        rampFlywheelUp();
        sleep(150);
        elevator.setPower(1);
        sleep(2000);
        elevator.setPower(0);
        sleep(150);
        rampFlywheelDown();
        flip.setPosition(0);
    }

    private void moveToLine(){
        driveTrain.setPower(0.8);
        while(light_ground.getLightDetected() <= LINE_THRESHOLD){
            idle();
        }
        driveTrain.setPower(0);
    }

    private void forward(){
        driveTrain.setPower(-0.3);
        sleep(350);
        driveTrain.stop();
        sleep(150);
    }

    protected void backward(){
        driveTrain.setPower(0.15);
        sleep(100);
        driveTrain.stop();
        sleep(150);
    }


    private void pressBlue(){                                 //These two seem opposite, it's pressing the color stated to change it to the other
        if(light_beacon.getRawLightDetected() <= BEACON_THRESHOLD){
            forward();
            backward();
        }
    }

    private void pressRed(){
        if(light_beacon.getRawLightDetected() > BEACON_THRESHOLD){
            forward();
            backward();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        pushL.setPower(0.7);
        pushR.setPower(0.7);
        sleep(1500);
        pushL.setPower(0);
        pushR.setPower(0);

        driveTrain.turn("left", 0.5*alliance);                   //Turns towards beacon
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

        autoShoot();                                    //Automatically shoots

        driveTrain.setPower(0.5);
        sleep(1000);
        driveTrain.stop();
        sleep(150);

        driveTrain.turn("right", 0.5*alliance);                   //Turns towards next beacon line
        sleep(500);                 //TODO ~90 deg towards next beacon line
        driveTrain.stop();
        sleep(150);

        moveToLine();                                   //Moves to next beacon line
        sleep(250);

        driveTrain.turn("left", 0.5*alliance);                   //Turns towards next beacon
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
    }
}
