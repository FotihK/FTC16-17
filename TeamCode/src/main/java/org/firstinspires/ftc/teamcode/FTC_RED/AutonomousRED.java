package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helper.AutonomousTemp;

/**
 * Created by fotih on 12/9/2016.
 */
@Autonomous(name = "Red", group = "Auton")
public class AutonomousRED extends AutonomousTemp {

    public void telemetry() {
        telemetry.addData("Beacon val: ", light_beacon.getRawLightDetected());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        sleep(1000);
        driveTrain.setPower(0.4);
        while ((light_ground.getRawLightDetected() < 2) && opModeIsActive()) {    //Runs until white beacon line is seen
            idle();
        }
        sleep(250);
        driveTrain.stop();
        sleep(500);
        driveTrain.turn("left", 0.7);
        while ((light_ground.getRawLightDetected() < 2) && opModeIsActive()) {    //Runs until white beacon line is seen
            idle();
        }
        //sleep(1000);
        driveTrain.stop();
        sleep(500);
        telemetry();
        //stop();
        if (light_beacon.getRawLightDetected() < 2.05) {
            pushL.setPosition(0.04);
            pushR.setPosition(0.04);
        } else {
            pushL.setPosition(0.9);
            pushR.setPosition(0.9);
        }
        telemetry();
        sleep(250);
        driveTrain.setPower(0.3);
        sleep(250);
        driveTrain.stop();
        sleep(500);
        driveTrain.setPower(-0.4);
        sleep(1000);
        driveTrain.stop();
        sleep(500);
        autoShoot();

    }
}
