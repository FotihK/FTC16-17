package org.firstinspires.ftc.teamcode.FTC_RED.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

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
                while (heading > newHeading && opModeIsActive()) {
                    idle();
                }
            } else {
                driveTrain.turn("left", power);
                while (heading < newHeading && opModeIsActive()) {
                    idle();
                }
            }
        } else {
            if ((heading + angle) > heading) {
                driveTrain.turn("right", power);
                while (heading < newHeading && opModeIsActive()) {
                    idle();
                }
            } else {
                driveTrain.turn("left", power);
                while (heading > newHeading && opModeIsActive()) {
                    idle();
                }
            }
        }
        driveTrain.stop();


    }

    private void genOffset(){
        int sum = 0;
        for(int i = 0; (i < 50) && (opModeIsActive()); i++){
            sum += gyro.getRotationFraction();
            sleep(15);
        }
        offset = sum / 50;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("right"), hardwareMap.dcMotor.get("left"));
        gyro = hardwareMap.gyroSensor.get("gyro");

        offset = 0;
        genOffset();
        lastTime = System.currentTimeMillis();
        heading = 0;

        gyroThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    integrateGyro();
                    telemetry.addData("Gyro Heading: ", heading);
                    telemetry.update();
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });

        gyroThread.start();

        double d_theta = 15;
        double calculated = 0;

        for(int i = 0; i < (720 / d_theta); i++){
            turn(0.5, d_theta);
            calculated += d_theta;
            telemetry.addData("Calculated heading: ", calculated);
            telemetry.update();
            sleep(1000);
        }
    }
}
