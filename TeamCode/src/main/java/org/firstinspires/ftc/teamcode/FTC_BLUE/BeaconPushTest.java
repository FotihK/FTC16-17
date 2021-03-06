package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


/**
 * Created by HP on 9/29/2016.
 */

//@Autonomous(name = "BeaconPushTest(Praful)", group = "BLUE")
public class BeaconPushTest extends LinearOpMode {
    private DriveTrain driveTrain;
    private LightSensor light;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double thresh = 1.94;
    private String last;

    public void initialize() {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("fL"), hardwareMap.dcMotor.get("fR"), hardwareMap.dcMotor.get("bL"),
                hardwareMap.dcMotor.get("bR"));
        light = hardwareMap.lightSensor.get("light_beacon");
        light.enableLed(false);
    }

    private void forward() {
        driveTrain.setPower(0.25);
        timer.reset();
        while (timer.time() < 450 && opModeIsActive()) {
            telemetry.addData("Raw: ", light.getRawLightDetected());
            telemetry.addData("Val: ", light.getLightDetected());
            telemetry.addData("pressing", "");
            telemetry.update();
            idle();
        }
        driveTrain.stop();
    }

    private void backward() {
        driveTrain.setPower(-0.15);
        timer.reset();
        while (timer.time() < 100 && opModeIsActive()) {
            telemetry.addData("Raw: ", light.getRawLightDetected());
            telemetry.addData("Val: ", light.getLightDetected());
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
            telemetry.addData("Raw: ", light.getRawLightDetected());
            telemetry.addData("Val: ", light.getLightDetected());
            if((timer.milliseconds()%100) >= 0 && (timer.milliseconds()%100) <= 6 ){
                if(light.getRawLightDetected() <= thresh) checks.add(true);
            }
        }

        int count = 0;

        for(boolean check : checks){
            if(check) count++;
        }

        if (light.getRawLightDetected() <= thresh && count >= 8) {
            forward();
            backward();
            last = "Blue";
        }
    }

    private void pressRed() {
        ArrayList<Boolean> checks = new ArrayList<>();
        timer.reset();
        while(timer.milliseconds() <= 1001 && opModeIsActive()){
            telemetry.addData("Raw: ", light.getRawLightDetected());
            telemetry.addData("Val: ", light.getLightDetected());
            if((timer.milliseconds()%100) >= 0 && (timer.milliseconds()%100) <= 6 ){
                if(light.getRawLightDetected() > thresh) checks.add(true);
            }
        }

        int count = 0;

        for(boolean check : checks){
            if(check) count++;
        }

        if (light.getRawLightDetected() > thresh && count >= 8) {
            forward();
            backward();
            last = "Red";
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        forward();
        backward();
        sleep(6000);
        while (opModeIsActive()) {
            pressRed();
            while (timer.time() < 6000 && opModeIsActive()) {
                telemetry.addData("Raw: ", light.getRawLightDetected());
                telemetry.addData("Val: ", light.getLightDetected());
                telemetry.addData("Waiting", "");
                telemetry.addLine(last);
                telemetry.update();
                idle();
            }
        }
    }
}
