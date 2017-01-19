package org.firstinspires.ftc.teamcode.FTC_BLUE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * The DriveTrain class provides a more compact way of controlling a pair of motors that are to work
 * in tandem.
 */

public class DriveTrain {
    private DcMotor left, right;

    public DriveTrain(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the direction of either (or both) motor(s)
     *
     * @param motor the name of the motor whose direction will be set (can be both)
     * @param dir   the direction which will be set
     * @see DcMotor#setDirection(DcMotorSimple.Direction)
     */
    @SuppressWarnings("WeakerAccess")
    public void setDirection(String motor, DcMotorSimple.Direction dir) {
        switch (motor.toLowerCase()) {
            case "left":
                left.setDirection(dir);
                break;
            case "right":
                right.setDirection(dir);
                break;
            case "both":
                left.setDirection(dir);
                right.setDirection(dir);
                break;
        }
    }

    /**
     * Sets the mode of either (or both) motor(s).
     *
     * @param motor the name of the motor whose mode will be set (can be both)
     * @param mode  the mode which will be set
     * @see DcMotor#setMode(DcMotor.RunMode)
     */
    @SuppressWarnings("WeakerAccess")
    public void setMode(String motor, DcMotor.RunMode mode) {
        switch (motor.toLowerCase()) {
            case "left":
                left.setMode(mode);
                break;
            case "right":
                right.setMode(mode);
                break;
            case "both":
                left.setMode(mode);
                right.setMode(mode);
                break;
        }
    }

    /**
     * Retrieves the y-stick values of the given gamepad and returns them provided they are not within dead zone
     *
     * @param gp Gamepad from which to retrieve values
     * @return an array of the y-stick values {left, right}
     * @see #tankDrive(Gamepad)
     * @see #arcadeDrive(Gamepad)
     */
    public double[] getGamepadYValues(Gamepad gp) {
        return new double[]{Math.abs(gp.left_stick_y) >= 0.025 ? -gp.left_stick_y : 0,
                Math.abs(gp.right_stick_y) >= 0.025 ? -gp.right_stick_y : 0};
    }

    /**
     * Retrieves the x-stick values of the given gamepad and returns them provided they are not within dead zone
     *
     * @param gp Gamepad from which to retrieve values
     * @return an array of the x-stick values {left, right}
     * @see #tankDrive(Gamepad)
     * @see #arcadeDrive(Gamepad)
     */
    public double[] getGamepadXValues(Gamepad gp) {
        return new double[]{Math.abs(gp.left_stick_x) >= 0.025 ? gp.left_stick_x : 0,
                Math.abs(gp.right_stick_x) >= 0.025 ? gp.right_stick_x : 0};
    }

    /**
     * Tank drive using given gamepad
     *
     * @param gp Gamepad used to drive the DriveTrain
     */
    public void tankDrive(Gamepad gp) {
        left.setPower(getGamepadYValues(gp)[0]);
        right.setPower(getGamepadYValues(gp)[1]);
    }

    /**
     * Arcade drive using right stick of given gamepad
     *
     * @param gp Gamepad used to drive the DriveTrain
     */
    public void arcadeDrive(Gamepad gp) {
        left.setPower(getGamepadYValues(gp)[1] + getGamepadXValues(gp)[1]);
        right.setPower(getGamepadYValues(gp)[1] - getGamepadXValues(gp)[1]);
    }

    /**
     * Set power of both motors
     *
     * @param pow power
     */
    public void setPower(double pow) {
        pow = Range.clip(pow, -1, 1);
        left.setPower(pow);
        right.setPower(pow);
    }

    /**
     * Quick stop method for both motors
     */
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    /**
     * Point turns in specified direction at specified power
     *
     * @param dir direction
     * @param pow power
     */
    public void turn(String dir, double pow) {
        pow = Range.clip(pow, -1, 1);
        switch (dir.toLowerCase()) {
            case "left":
                left.setPower(-pow);
                right.setPower(pow);
                break;
            default:
                left.setPower(pow);
                right.setPower(-pow);
        }
    }

}
