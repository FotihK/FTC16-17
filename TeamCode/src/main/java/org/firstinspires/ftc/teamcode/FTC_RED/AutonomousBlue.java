package org.firstinspires.ftc.teamcode.FTC_RED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTC_RED.Helper.AutonomousTemp;

/**
 * Created by fotih on 12/9/2016.
 */
@Autonomous(name = "Blue", group = "RED")
public class AutonomousBLUE extends AutonomousTemp {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = -1;
        super.runOpMode();
    }
}
