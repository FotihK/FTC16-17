package org.firstinspires.ftc.teamcode.FTC_GREEN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

public class BlueAutonomous extends LinearOpMode {

    private DcMotor one, two, three, four;
    private LightSensor light_beacon, light_ground;
    protected int alliance = 1;
    private double lightThresh = 1.8;
    private final double BEACON_THRESHOLD = 1.875;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    private void initialize(){
        one = hardwareMap.dcMotor.get("one");       //Negative + CW
        two = hardwareMap.dcMotor.get("two");       //Negative + CW
        three = hardwareMap.dcMotor.get("three");   //Positive + CCW
        four = hardwareMap.dcMotor.get("four");     //Positive + CCW
        light_beacon = hardwareMap.lightSensor.get("light_beacon");

    }

    private void straight(double power){
        power = Range.clip(power, -1, 1);
        one.setPower(-power);
        two.setPower(-power);
        three.setPower(power);
        four.setPower(power);
    }

    private void strafe(double power){
        power = Range.clip(power, -1, 1);
        one.setPower(-power);
        two.setPower(power);
        three.setPower(power);
        four.setPower(-power);
    }

    private void turn(String dir, double power){
        power = Range.clip(power, 0, 1);
        switch(dir.toLowerCase()){
            case "left":
                one.setPower(power);
                two.setPower(power);
                three.setPower(power);
                four.setPower(power);
                break;
            default:
                one.setPower(-power);
                two.setPower(-power);
                three.setPower(-power);
                four.setPower(-power);
                break;
        }
    }

    private void stopDrive(){
        one.setPower(0);
        two.setPower(0);
        three.setPower(0);
        four.setPower(0);
    }

    private void forward() {
        straight(0.4);
        sleep(100);
        stopDrive();
        sleep(75);
    }

    private void backward() {
        straight(-0.2);
        sleep(75);
        stopDrive();
        sleep(75);
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

        if (count >= (int)((checks.size() * 0.75) + 0.5) ) {
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

        if (count >= (int)((checks.size() * 0.75) + 0.5)) {
            forward();
            backward();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        straight(0.5);
        while(light_ground.getLightDetected() < lightThresh && opModeIsActive()){
            idle();
        }
        stopDrive();


        turn("right", 0.5 * alliance);
        sleep(450);
        stopDrive();

        straight(0.5);
        sleep(450);
        stopDrive();

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        if (alliance == 1) pressRed();
        else pressBlue();

        straight(-0.2);
        sleep(150);
        stopDrive();

        turn("left", 0.5 * alliance);
        sleep(910);
        stopDrive();

        straight(0.5);
        while(light_ground.getLightDetected() < lightThresh && opModeIsActive()){
            idle();
        }
        stopDrive();

        turn("right", 0.5 * alliance);
        sleep(910);
        stopDrive();

        straight(0.5);
        sleep(450);
        stopDrive();

        backward();                                     //Prepares for reading, and then goes ahead and does it if necessary
        if (alliance == 1) pressRed();
        else pressBlue();

    }
}
