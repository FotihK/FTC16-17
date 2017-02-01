package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.AutonomousTemp;

/**
 * Created by fotih on 12/9/2016.
 */
@Autonomous(name = "Red", group = "RED")
public class AutonomousRED extends AutonomousTemp {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = 1;
        super.runOpMode();

    }
}
