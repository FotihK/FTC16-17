package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by HP on 10/11/2016.
 */

class SynchedMotors {
    private DcMotor m1, m2;
    private double ratio;
    SynchedMotors(DcMotor m1, DcMotor m2, float ratio){
        this.m1 = m1;
        this.m2 = m2;
        this.ratio = Range.clip(ratio,-1,1);
    }

    public void setRatio(double ratio){
        this.ratio = Range.clip(ratio,-1,1);
    }

    void setPower(double pow){
        double power = Range.clip(pow, 0, 1);
        m1.setPower(power);
        m2.setPower(power * ratio);
    }

}
