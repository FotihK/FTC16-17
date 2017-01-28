package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Created by Siva on 11/9/2016.
 */
//@Autonomous(name="Auton-Straight", group="BLUE")
public class Autonomous_Straight extends Autonomous_Red {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        flip.setPosition(0.5);

        driveTrain.setPower(-0.5);
        sleep(1700);
        driveTrain.stop();

        autoShoot();
    }
}
