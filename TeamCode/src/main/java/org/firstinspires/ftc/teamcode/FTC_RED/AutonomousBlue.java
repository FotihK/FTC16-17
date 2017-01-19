package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.AutonomousTemp;

/**
 * Created by fotih on 12/9/2016.
 */
@Autonomous(name = "Blue", group = "Auton")
public class AutonomousBLUE extends AutonomousTemp {

    //TODO: Tweak timings to make everything run smoothly
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();                                   //initializes, waits for start
        alliance = -1;
        waitForStart();
        pushL.setPosition(servoEndPositions[0]);        //puts servos down
        pushR.setPosition(servoEndPositions[1]);

        sleep(5000);
        moveToLine();                                   //Waits 5 seconds, moves to first beacon line
        sleep(250);

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

        autoShoot();                                    //Automatically shoots, see AutonomouTemp

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

        driveTrain.setPower(-0.5);                      //Backs away from the beacon to get ready to shoot preloads
        sleep(1000);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(150);

        driveTrain.turn("left", 0.5*alliance);                   //Turns towards front of ramp
        sleep(500);                 //TODO ~90 deg towards front of ramp
        driveTrain.stop();
        pushL.setPosition(servoStartPositions[0]);        //puts servos up
        pushR.setPosition(servoStartPositions[1]);
        sleep(150);

        driveTrain.setPower(0.5);                      //Moves towards front of ramp
        sleep(2500);                //TODO find timing for optimal distance
        driveTrain.stop();
        sleep(150);

        driveTrain.turn("right", 0.5*alliance);                   //Turns towards ramp
        sleep(500);                 //TODO ~45 deg towards ramp
        driveTrain.stop();
        sleep(150);

        sleep(500);

        driveTrain.setPower(0.7);       //Climb up onto ramp and keep climbing
        while (opModeIsActive()) {
            sleep(1);
        }
    }

}
