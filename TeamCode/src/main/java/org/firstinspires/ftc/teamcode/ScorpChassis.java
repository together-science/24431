package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class ScorpChassis {
    private DcMotor lf=null, rf=null, lb=null, rb=null;
    private IMU imu = null;
    private SparkFunOTOS otos = null;

    ScorpChassis(HardwareMap hm, String lfName, String rfName, String lbName, String rbName, String otosName, String imuName) {
        this.lf = hm.get(DcMotor.class, lfName);
        this.rf = hm.get(DcMotor.class, rbName);
        this.lb = hm.get(DcMotor.class, lbName);
        this.rb = hm.get(DcMotor.class, rbName);
        this.otos = hm.get(SparkFunOTOS.class, otosName);
        this.imu = hm.get(IMU.class, imuName);
    }
}
