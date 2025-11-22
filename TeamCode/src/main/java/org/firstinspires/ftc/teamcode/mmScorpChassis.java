package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class mmScorpChassis {
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftBackWheel;
    private DcMotor rightBackWheel;
    private SparkFunOTOS otos;
    private IMU imu;
    private LinearOpMode op;

    mmScorpChassis(LinearOpMode op, String lfName, String rfName, String lbName, String rbName, String otosName, String imuName) {
        try {
            this.leftFrontWheel = op.hardwareMap.get(DcMotor.class, lfName);
            this.rightFrontWheel = op.hardwareMap.get(DcMotor.class, rfName);
            this.leftBackWheel = op.hardwareMap.get(DcMotor.class, lbName);
            this.rightBackWheel = op.hardwareMap.get(DcMotor.class, rbName);
            this.otos = op.hardwareMap.get(SparkFunOTOS.class, otosName);
            this.imu = op.hardwareMap.get(IMU.class, imuName);
            this.op = op;
        } catch (Exception ignored) {
        }
    }
    void init(){
        if (leftFrontWheel != null && leftBackWheel != null && rightFrontWheel != null && rightBackWheel != null) {
            leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
            rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
            leftBackWheel.setDirection(DcMotor.Direction.FORWARD);
            rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
            leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(imu != null) {
            //The next two lines define Hub orientation.
            // The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
            // To Do:  EDIT these two lines to match YOUR mounting configuration.
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        }
        if (otos != null) {
            otos.setLinearUnit(DistanceUnit.INCH);
            otos.setAngularUnit(AngleUnit.DEGREES);
            SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
            otos.setOffset(offset);
            otos.setLinearScalar(1.0434);
            otos.setAngularScalar(1.0); // So does this
            otos.calibrateImu();
            otos.resetTracking();
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
            otos.setPosition(currentPosition);
            SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
            SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
            otos.getVersionInfo(hwVersion, fwVersion);
        }
    }
    void stop(){
        leftFrontWheel.setPower(0);
        rightFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightBackWheel.setPower(0);
    }
    void setPowerAllWheels(){

    }
    void testDrive(){
        leftFrontWheel.setPower(0.5);
        rightFrontWheel.setPower(0.5);
        leftBackWheel.setPower(0.5);
        rightBackWheel.setPower(0.5);
        op.sleep(2000);
        stop();
    }
}