package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HP on 9/29/2016.
 */

@TeleOp(name="GyroTest",group = "Tests")
public class GyroTest extends OpMode {
    private GyroSensor gyro;
    private boolean[] toggle = new boolean[2];
    private double offset, heading;
    private long lastTime;


    @Override
    public void init() {
        gyro = hardwareMap.gyroSensor.get("gyro");

        offset = gyro.getRotationFraction();
        lastTime = System.currentTimeMillis();

    }

    private void integrateGyro(){
        long currTime = System.currentTimeMillis();
        if(Math.abs((gyro.getRotationFraction() - offset)) * ((currTime - lastTime)) > 0.025)
        heading += ((gyro.getRotationFraction() - offset)) * ((currTime - lastTime));
        lastTime = currTime;

    }

    @Override
    public void loop() {
        if(gamepad1.y && !toggle[0]){
            offset = 0;
            toggle[0] = true;
        } else if(!gamepad1.y) toggle[0] = false;

        integrateGyro();

        telemetry.addData("Gyro Fractional:", gyro.getRotationFraction() - offset);
        telemetry.addData("Offset: ", offset);
        telemetry.addData("Heading: ", heading);
    }
}
