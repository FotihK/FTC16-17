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
@Autonomous(name="BlueSide-PrafulBot", group="Autonomous")
public class Auto_BlueSide_PrafulBot extends Auto_RedSide_PrafulBot {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = -1;
        super.runOpMode();
    }
}
