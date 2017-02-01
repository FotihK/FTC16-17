package org.firstinspires.ftc.teamcode.FTC_RED.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC_RED.Helper.DriveTrain;

@Autonomous(name="Gyro Turn Test", group = "Tests")
public class GyroTurning extends LinearOpMode {

    private DriveTrain driveTrain;
    private GyroSensor gyro;
    private double offset, heading;
    private long lastTime;
    private Thread gyroThread;

    private void integrateGyro() {
        long currTime = System.currentTimeMillis();
        if (Math.abs((gyro.getRotationFraction() - offset)) * ((currTime - lastTime)) > 0.025)
            heading += ((gyro.getRotationFraction() - offset)) * ((currTime - lastTime));
        lastTime = currTime;
        if (heading >= 360.5) heading -= 360;
        if (heading < -0.5) heading += 360;
    }

    private void turn(double power, double angle) {
        double newHeading = heading + angle;
        boolean wrap = false;
        if (newHeading >= 360.5) {
            newHeading -= 360;
            wrap = true;
        }
        if (newHeading < -0.5) {
            newHeading += 360;
            wrap = true;
        }

        if (wrap) {
            if ((heading + angle) > heading) {
                driveTrain.turn("right", power);
                while (heading > newHeading && heading < (newHeading + 10) && opModeIsActive()) {
                    idle();
                }
            } else {
                driveTrain.turn("left", power);
                while (heading < newHeading && heading < (newHeading + 10) && opModeIsActive()) {
                    idle();
                }
            }
        } else {
            if ((heading + angle) > heading) {
                driveTrain.turn("right", power);
                while (heading < newHeading && heading < newHeading + 10 && opModeIsActive()) {
                    idle();
                }
            } else {
                driveTrain.turn("left", power);
                while (heading > newHeading && heading < newHeading + 10 && opModeIsActive()) {
                    idle();
                }
            }
        }
        driveTrain.stop();


    }

    private void genOffset(){
        double sum = 0;
        for(int i = 0; (i < 50); i++){
            sum += gyro.getRotationFraction();
            sleep(15);
        }
        offset = sum / 50.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("right"), hardwareMap.dcMotor.get("left"));
        gyro = hardwareMap.gyroSensor.get("gyro");
        sleep(500);

        offset = gyro.getRotationFraction();
        genOffset();

        lastTime = System.currentTimeMillis();
        heading = 0;
        telemetry.addData("Offset:", offset);
        telemetry.update();

        waitForStart();

        gyroThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    integrateGyro();
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });

        gyroThread.start();

        double d_theta = 22.5;
        double calculated = 0;

        for(int i = 0; i < (720 / d_theta); i++){
            turn(0.25, d_theta);
            calculated += d_theta;
            telemetry.addData("Calculated heading: ", calculated);
            telemetry.addData("Heading :", heading);
            telemetry.update();
            sleep(3000);
        }
    }
}
