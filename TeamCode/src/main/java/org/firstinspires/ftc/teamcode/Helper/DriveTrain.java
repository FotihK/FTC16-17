package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by HP on 12/30/2016.
 */

public class DriveTrain {
    private DcMotor left,right;
    private Gamepad gp;
    public DriveTrain(DcMotor left, DcMotor right, Gamepad gp){
        this.left = left;
        this.right = right;
        this.gp = gp;
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public double[] getGamepadYValues(){
        return new double[]{Math.abs(gp.left_stick_y) >= 0.025 ? -gp.left_stick_y : 0,
                Math.abs(gp.right_stick_y) >= 0.025 ? -gp.right_stick_y : 0};
    }

    public double[] getGamepadXValues(){
        return new double[]{Math.abs(gp.left_stick_x) >= 0.025 ? gp.left_stick_x : 0,
                Math.abs(gp.right_stick_x) >= 0.025 ? gp.right_stick_x : 0};
    }
    
    public void tankDrive(){
        left.setPower(getGamepadYValues()[0]);
        right.setPower(getGamepadYValues()[1]);
    }
    
    public void arcadeDrive(){
        left.setPower(getGamepadYValues()[1] + getGamepadXValues()[1]);
        right.setPower(getGamepadYValues()[1] - getGamepadXValues()[1]);
    }
    
    public void straight(double pow){
        left.setPower(pow);
        right.setPower(pow);
    }
    
    public void stop(){
        left.setPower(0);
        right.setPower(0);
    }
    
    public void turn(String dir, double pow){
        switch(dir.toLowerCase()){
            case "l":
                left.setPower(-pow);
                right.setPower(pow);
                break;
            default:
                left.setPower(pow);
                right.setPower(-pow);
        }
    }
    
}
