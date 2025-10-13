package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo; // Not used yet <--

public class ScorpChassis {
    private final DcMotor lf;
    private final DcMotor rf;
    private final DcMotor lb;
    private final DcMotor rb;
    private final SparkFunOTOS otos;
    private final IMU imu;

    ScorpChassis(HardwareMap hm, String lfName, String rfName, String lbName, String rbName, String otosName, String imuName) {
        this.lf = hm.get(DcMotor.class, lfName);
        this.rf = hm.get(DcMotor.class, rbName);
        this.lb = hm.get(DcMotor.class, lbName);
        this.rb = hm.get(DcMotor.class, rbName);
        this.otos = hm.get(SparkFunOTOS.class, otosName);
        this.imu = hm.get(IMU.class, imuName);
    }

    boolean init(){
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return true;
    }
}
