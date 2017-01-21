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
    private DcMotor fL, fR, bL, bR;

    public DriveTrain(DcMotor fL, DcMotor fR, DcMotor bL, DcMotor bR) {
        this.fL = bR;
        this.fR = bL;
        this.bL = fR;
        this.bR = fL;

        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            case "fl":
                fL.setDirection(dir);
                break;
            case "fr":
                fR.setDirection(dir);
                break;
            case "front":
                fL.setDirection(dir);
                fR.setDirection(dir);
                break;
            case "bl":
                bL.setDirection(dir);
                break;
            case "br":
                bR.setDirection(dir);
                break;
            case "back":
                bL.setDirection(dir);
                bR.setDirection(dir);
                break;
            case "all":
                fL.setDirection(dir);
                fR.setDirection(dir);
                bL.setDirection(dir);
                bR.setDirection(dir);
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
            case "fl":
                fL.setMode(mode);
                break;
            case "fr":
                fR.setMode(mode);
                break;
            case "front":
                fL.setMode(mode);
                fR.setMode(mode);
                break;
            case "bl":
                bL.setMode(mode);
                break;
            case "br":
                bR.setMode(mode);
                break;
            case "back":
                bL.setMode(mode);
                bR.setMode(mode);
                break;
            case "all":
                fL.setMode(mode);
                fR.setMode(mode);
                bL.setMode(mode);
                bR.setMode(mode);
                break;
        }
    }

    /**
     * Retrieves the y-stick values of the given gamepad and returns them provided they are not within dead zone
     *
     * @param gp Gamepad from which to retrieve values
     * @return an array of the y-stick values {fL, fR}
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
     * @return an array of the x-stick values {fL, fR}
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
        fL.setPower(getGamepadYValues(gp)[0]);
        bL.setPower(getGamepadYValues(gp)[0]);
        fR.setPower(getGamepadYValues(gp)[1]);
        bR.setPower(getGamepadYValues(gp)[1]);
    }

    /**
     * Arcade drive using fR stick of given gamepad
     *
     * @param gp Gamepad used to drive the DriveTrain
     */
    public void arcadeDrive(Gamepad gp) {
        fL.setPower(getGamepadYValues(gp)[1] + getGamepadXValues(gp)[1]);
        fR.setPower(getGamepadYValues(gp)[1] - getGamepadXValues(gp)[1]);
        bL.setPower(getGamepadYValues(gp)[1] + getGamepadXValues(gp)[1]);
        bR.setPower(getGamepadYValues(gp)[1] - getGamepadXValues(gp)[1]);
    }

    /**
     * Set power of both motors
     *
     * @param pow power
     */
    public void setPower(double pow) {
        pow = Range.clip(pow, -1, 1);
        fL.setPower(pow);
        fR.setPower(pow);
        bL.setPower(pow);
        bR.setPower(pow);
    }

    /**
     * Quick stop method for both motors
     */
    public void stop() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
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
                fL.setPower(-pow);
                fR.setPower(pow);
                bL.setPower(-pow);
                bR.setPower(pow);
                break;
            default:
                fL.setPower(pow);
                fR.setPower(-pow);
                bL.setPower(pow);
                bR.setPower(-pow);
        }
    }

}
