package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;


/**
 * Created by Siva on 11/9/2016.
 */
@Autonomous(name = "RedSide-PrafulBot", group = "BLUE")
public class Autonomous_Red extends LinearOpMode {
    protected DriveTrain driveTrain;
    private DcMotor elevator, flywheel;
    protected Servo flip, load;
    private double flyPower = 0;
    private LightSensor light_ground, light_beacon;
    private final double BEACON_THRESHOLD = 1.94;            //Less than for blue, greater than for red
    private final double LINE_THRESHOLD = 0.33;
    int alliance = 1;                   //1 for red, -1 for blue
    private double turnPow = 0.35;
    private ElapsedTime sleepTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void initialize() {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("fL"), hardwareMap.dcMotor.get("fR"), hardwareMap.dcMotor.get("bL"),
                hardwareMap.dcMotor.get("bR"));

        elevator = hardwareMap.dcMotor.get("elevator");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        flip = hardwareMap.servo.get("flip");
        load = hardwareMap.servo.get("load");
        light_ground = hardwareMap.lightSensor.get("light_ground");
        light_beacon = hardwareMap.lightSensor.get("light_beacon");

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flip.setDirection(Servo.Direction.FORWARD);
        load.setDirection(Servo.Direction.FORWARD);

        flip.setPosition(0);
        load.setPosition(0);

        light_ground.enableLed(true);
        light_beacon.enableLed(false);
    }

    protected void sleep(int milliseconds) {
        sleepTimer.reset();
        while (sleepTimer.milliseconds() <= milliseconds && opModeIsActive()) {
            idle();
        }
    }


    private void rampFlywheelUp() throws InterruptedException {
        while (flyPower < 1) {
            flyPower = Range.clip(flyPower + 0.20, 0, 1);
            flywheel.setPower(flyPower);
            sleep(450);
        }
    }

    private void rampFlywheelDown() throws InterruptedException {
        while (flyPower > 0) {
            flyPower = Range.clip(flyPower - 0.15, 0, 1);
            flywheel.setPower(flyPower);
            sleep(500);
        }
    }

    protected void autoShoot() throws InterruptedException {
        rampFlywheelUp();
        sleep(75);
        elevator.setPower(1);
        sleep(2000);
        elevator.setPower(0);
        sleep(75);
        rampFlywheelDown();
        flip.setPosition(0);
    }

    private void moveToLine() {
        driveTrain.setPower(0.8);
        while (light_ground.getLightDetected() <= LINE_THRESHOLD && opModeIsActive()) {
            idle();
        }
        sleep(150);
        driveTrain.setPower(0);
    }

    private void forward() {
        driveTrain.setPower(0.4);
        timer.reset();
        while (timer.time() < 350 && opModeIsActive()) {
            telemetry.addData("Raw: ", light_beacon.getRawLightDetected());
            telemetry.addData("Val: ", light_beacon.getLightDetected());
            telemetry.addData("pressing", "");
            telemetry.update();
            idle();
        }
        driveTrain.stop();
    }

    private void backward() {
        driveTrain.setPower(-0.2);
        timer.reset();
        while (timer.time() < 150 && opModeIsActive()) {
            telemetry.addData("Raw: ", light_beacon.getRawLightDetected());
            telemetry.addData("Val: ", light_beacon.getLightDetected());
            telemetry.addData("backing", "");
            telemetry.update();
            idle();
        }
        driveTrain.stop();
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

        if (light_beacon.getRawLightDetected() <= BEACON_THRESHOLD && count >= 8) {
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

        if (light_beacon.getRawLightDetected() > BEACON_THRESHOLD && count >= 8) {
            forward();
            backward();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        elevator.setPower(1);
        moveToLine();
        elevator.setPower(0);
        flip.setPosition(0.55);
        driveTrain.turn("left", turnPow * alliance);                   //Turns towards beacon
        //sleep(500);                 //TODO towards beacon
        while (light_ground.getLightDetected() <= LINE_THRESHOLD && opModeIsActive()) {
            idle();
        }
        driveTrain.stop();
        elevator.setPower(-0.8);
        sleep(150);
        elevator.setPower(0);

        driveTrain.setPower(0.5);                       //Ram into beacon to activate
        sleep(750);                //TODO minimize
        driveTrain.stop();
        sleep(50);

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        sleep(2750);
        if (alliance == 1) pressBlue();
        else pressRed();
        telemetry.clearAll();

        sleep(100);

        driveTrain.setPower(-0.4);                      //Backs away from the beacon to get ready to shoot preloads
        sleep(950);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(75);

        driveTrain.turn("left", turnPow * alliance);
        sleep(145);
        driveTrain.stop();
        sleep(75);

        autoShoot();                                    //Automatically shoots
        //sleep(2000);
        sleep(125);

        driveTrain.turn("right", turnPow * alliance);
        sleep(145);
        driveTrain.stop();
        sleep(75);

        driveTrain.setPower(0.4);                       //Goes back towards beacon
        sleep(825);
        driveTrain.stop();
        sleep(75);

        driveTrain.turn("right", turnPow * alliance);                   //Turns towards next beacon line
        sleep(910);                 //TODO ~90 deg towards next beacon line
        driveTrain.stop();
        sleep(75);

        moveToLine();                                   //Moves to next beacon line

        driveTrain.setPower(0.4);
        sleep(70);
        driveTrain.stop();
        sleep(75);

        driveTrain.turn("left", turnPow * alliance);                   //Turns towards next beacon
        //sleep(925);                 //TODO ~90 deg towards next beacon
        while (light_ground.getLightDetected() <= LINE_THRESHOLD && opModeIsActive()) {
            idle();
        }
        driveTrain.stop();
        sleep(75);
        driveTrain.turn("right", turnPow * alliance);
        sleep(100);

        driveTrain.setPower(0.5);                       //Ram into beacon to activate
        sleep(750);                //TODO minimize
        driveTrain.stop();
        sleep(75);

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        sleep(3000);
        if (alliance == 1) pressBlue();
        else pressRed();
        telemetry.clearAll();

    }
}