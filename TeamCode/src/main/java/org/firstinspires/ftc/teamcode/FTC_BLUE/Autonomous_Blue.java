package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Created by Siva on 11/9/2016.
 */
@Autonomous(name="BlueSide-PrafulBot", group="BLUE")
public class Autonomous_Blue extends Autonomous_Red {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = -1;
        super.runOpMode();
    }
}
