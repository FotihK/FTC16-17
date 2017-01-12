package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.DriveTrain;

/**
 * Created by HP on 9/29/2016.
 */

@Autonomous(name="BeaconPushTest",group = "Tests")
public class BeaconPushTest extends LinearOpMode {
    private DriveTrain driveTrain;
    private LightSensor light;
    private Servo pushL, pushR;
    private double[] servoEndPositions = {0.55, 0.35};      //Left, Right
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int count = 0;
    private boolean k = false;

    public void initialize() {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("left"), hardwareMap.dcMotor.get("right"));
        pushL = hardwareMap.servo.get("pushL");
        pushR = hardwareMap.servo.get("pushR");
        pushL.setDirection(Servo.Direction.FORWARD);
        pushR.setDirection(Servo.Direction.FORWARD);
        pushL.setPosition(servoEndPositions[0]);
        pushR.setPosition(servoEndPositions[1]);
        light = hardwareMap.lightSensor.get("light_beacon");
        light.enableLed(false);
    }

    private void forward(){
        driveTrain.setPower(-0.3);
        timer.reset();
        while(timer.time() < 350){
            telemetry.addData("Raw: ", light.getRawLightDetected());
            telemetry.addData("Val: ", light.getLightDetected());
            telemetry.addData("pressing","");
            telemetry.update();
            idle();
        }
        driveTrain.stop();
    }

    private void backward(){
        driveTrain.setPower(0.15);
        timer.reset();
        while(timer.time() < 100){
            telemetry.addData("Raw: ", light.getRawLightDetected());
            telemetry.addData("Val: ", light.getLightDetected());
            telemetry.addData("backing", "");
            telemetry.update();
            idle();
        }
        driveTrain.stop();
    }


    private void pressBlue(){
        if(light.getRawLightDetected() <= 1.875){
            forward();
            backward();
            count++;
        }
    }

    private void pressRed(){
        if(light.getRawLightDetected() > 1.875){
            forward();
            backward();
            count++;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){

            if((count % 2) == 0) pressBlue();
            else pressRed();
            while(timer.time() < 6000 && opModeIsActive()){
                telemetry.addData("Raw: ", light.getRawLightDetected());
                telemetry.addData("Val: ", light.getLightDetected());
                telemetry.addData("Waiting", "");
                telemetry.update();
                idle();

            }
        }
    }
}
