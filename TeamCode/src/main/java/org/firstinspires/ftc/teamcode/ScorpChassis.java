package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ScorpChassis implements RobotChassis {
    private DcMotor      lf;
    private DcMotor      rf;
    private DcMotor      lb;
    private DcMotor      rb;
    private SparkFunOTOS otos;
    private final IMU          imu;
    private final LinearOpMode op;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7*(24.0/35.0);
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_CORRECTION       = 1.0;
    static final double     DRIVE_SPEED_FAST        = 0.4;
    static final double     DRIVE_SPEED_NORMAL      = 0.2;
    static final double     DRIVE_SPEED_SLOW        = 0.1;
    static final double     TURN_SPEED              = 0.5;
    static final double     HEADING_THRESHOLD       = 1.0;
    static final double     P_TURN_GAIN             = 0.10;
    static final double     P_DRIVE_GAIN            = 0.03;
    static final double     ACCURACY                = 0.3; // inches accuracy for moveTo()
    static final boolean    DEBUG                   = true;

    ScorpChassis(LinearOpMode op, String lfName, String rfName, String lbName, String rbName, String otosName, String imuName) {

        this.imu = op.hardwareMap.get(IMU.class, imuName);
        this.op = op;

        try {
            this.lf = op.hardwareMap.get(DcMotor.class, lfName);
            this.rf = op.hardwareMap.get(DcMotor.class, rfName);
            this.lb = op.hardwareMap.get(DcMotor.class, lbName);
            this.rb = op.hardwareMap.get(DcMotor.class, rbName);
            this.otos = op.hardwareMap.get(SparkFunOTOS.class, otosName);
        } catch (Exception ignored) {
        }


    }
    void init(){
        if (lf != null && lb != null && rf != null && rb != null) {
            lf.setDirection(DcMotor.Direction.FORWARD);
            rf.setDirection(DcMotor.Direction.REVERSE);
            lb.setDirection(DcMotor.Direction.FORWARD);
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
        }
        //The next two lines define Hub orientation.
        // The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
        // To Do:  EDIT these two lines to match YOUR mounting configuration.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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

    //High level - Simple
    void moveTo(double x, double y, double driveSpeed){
        if (lf == null || lb == null || rf == null || rb == null || otos == null) {
            return;
        }
        double distance = 1000;
        double fh = getHeading(); // initial and final heading
        double dx = 0;
        double dy = 0;
        double h = 0;
        SparkFunOTOS.Pose2D pos = null;

        while(Math.abs(distance) > ScorpChassis.ACCURACY && op.opModeIsActive()) {
            pos = getPosition();

            dx = x - pos.x;
            dy = y - pos.y;
            distance = Math.sqrt(dx*dx+dy*dy);

            // lower driveSpeed as we get closer, and return to desired heading
            if (distance < 5) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_SLOW);
                h = fh;
            } else if (distance < 10) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_NORMAL);
                h = fh;
            } else {
                // try to face the target
                h = headingFromRelativePosition(dx, dy);
                h = normalizeAngle(h);
            }


            if (ScorpChassis.DEBUG) {
                this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
                this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
                this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
                this.op.telemetry.addData("Desired heading:", "%.4f", h);
                this.op.telemetry.addData("Current heading:", "%.4f", pos.h);
                this.op.telemetry.update();
            }

            _startDriveStraight(driveSpeed, h);
        }
        stop();
        while (ScorpChassis.DEBUG && op.opModeIsActive()) {
            this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
            this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
            this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
            this.op.telemetry.addData("Desired heading:", "%.4f", h);
            this.op.telemetry.addData("Current heading:", "%.4f", pos.h);
            this.op.telemetry.update();
        }
    }
    public SparkFunOTOS.Pose2D getPosition() {
        if (otos == null) {
            return null;
        }
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        return new SparkFunOTOS.Pose2D(-pos.x, -pos.y, pos.h);
    }
    public String getPositionString(){
        if (otos == null) {
            return "unknown";
        }
        SparkFunOTOS.Pose2D pos = getPosition();
        return "X:"+Math.round(pos.x*100)/100+", Y:"+Math.round(pos.y*100)/100+", H:"+Math.round(pos.h*100)/100;
    }

    //Medium level - Intermediate
    // todo: this won't work, the while loop is for encoders
    void driveStraight(double maxDriveSpeed, double distance, double heading) {
        throw new UnsupportedOperationException();
//        int lfTarget, rfTarget, lbTarget, rbTarget;
//        double turnSpeed;
//        if (op.opModeIsActive()) {
//            int moveCounts = (int)(distance * COUNTS_PER_INCH);
//            lfTarget = lf.getCurrentPosition() + moveCounts;
//            rfTarget = rf.getCurrentPosition() + moveCounts;
//            lbTarget = lb.getCurrentPosition() + moveCounts;
//            rbTarget = rb.getCurrentPosition() + moveCounts;
//            lf.setTargetPosition(lfTarget);
//            rf.setTargetPosition(rfTarget);
//            lb.setTargetPosition(lbTarget);
//            rb.setTargetPosition(rbTarget);
//
//            _moveRobot(Math.abs(maxDriveSpeed), 0);
//
//            // todo: fix this
//            while (op.opModeIsActive() && (lf.isBusy() && rf.isBusy())){
//                turnSpeed = _getSteeringCorrection(heading, P_DRIVE_GAIN);
//                if (distance < 0){
//                    turnSpeed *= -1.0;
//                }
//                _moveRobot(0, turnSpeed);
//            }
//
//            _moveRobot(0, 0);
//        }

    }


    void testStrafe(){
        lf.setPower(1.1);
        lb.setPower(1.1);

        rf.setPower(0.9);
        rb.setPower(0.9);
    }


    @Override
    // todo: this won't work, the while loop is for encoders
    public void strafeDistance(final double maxDriveSpeed, final double distance, final double heading) {
        throw new UnsupportedOperationException();
//        int lfTarget, rfTarget, lbTarget, rbTarget;
//        double turnSpeed;
//        if (op.opModeIsActive()) {
//
//            double axial   = Math.cos(Math.PI/180*heading);  // Note: pushing stick forward gives negative value
//            double lateral =  -Math.sin(Math.PI/180*heading);
//
//            double leftFrontPower  = (axial + lateral);
//            double rightFrontPower = (axial - lateral);
//            double leftBackPower   = (axial - lateral);
//            double rightBackPower  = (axial + lateral);
//
//            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));
//
//            if (max > 1.0) {
//                leftFrontPower /= max;
//                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
//            }
//
//            leftFrontPower *= maxDriveSpeed;
//            rightFrontPower *= maxDriveSpeed;
//            leftBackPower *= maxDriveSpeed;
//            rightBackPower *= maxDriveSpeed;
//
//            int moveCounts = (int)(distance * COUNTS_PER_INCH * (1+STRAFE_CORRECTION*Math.abs(lateral)));
//            lfTarget = lf.getCurrentPosition() + (int)Math.signum(leftFrontPower)*moveCounts;
//            rfTarget = rf.getCurrentPosition() +  (int)Math.signum(rightFrontPower)*moveCounts;
//            lbTarget = lb.getCurrentPosition() +  (int)Math.signum(leftBackPower)*moveCounts;
//            rbTarget = rb.getCurrentPosition() +  (int)Math.signum(rightBackPower)*moveCounts;
//
//            lf.setTargetPosition(lfTarget);
//            rf.setTargetPosition(rfTarget);
//            lb.setTargetPosition(lbTarget);
//            rb.setTargetPosition(rbTarget);
//
//            double robotHeading = getHeading();
//
//            _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);
//
//            while (op.opModeIsActive() &&
//                    (lf.isBusy() && rf.isBusy())) {
//
//                turnSpeed = _getSteeringCorrection(robotHeading, P_DRIVE_GAIN);
//
//                if (distance < 0)turnSpeed*=-1;
//                _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, turnSpeed);
//            }
//            _moveRobot(0, 0);
//        }

    }

    @Override
    public void turnToHeading(double maxTurnSpeed, double heading) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turnSpeed;
        _getSteeringCorrection(heading, P_DRIVE_GAIN);
        double current = getHeading();
        double headingError = heading - current;
        while (op.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = _getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            if (headingError < 5) {
                turnSpeed = Range.clip(turnSpeed, -0.1, 0.1);
            } else if (headingError < 15) {
                turnSpeed = Range.clip(turnSpeed, -0.3, 0.3);
            }
            _moveRobot(0, turnSpeed);
            current = getHeading();
            headingError = heading - current;
        }
        stop();
    }

    private void _startDriveStraight(double maxDriveSpeed, double h) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turnSpeed = _getSteeringCorrection(h, P_TURN_GAIN);
        _moveRobot(Math.abs(maxDriveSpeed), turnSpeed);
    }

    public double getHeading() {
        // if we have OTOS, use it.
        if (otos != null) {
            return getPosition().h;
        }
        // otherwise, use IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    //Low level - Advance
    private double _getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = desiredHeading - getHeading();

        headingError = normalizeAngle(headingError);

         return Range.clip(headingError * proportionalGain, -1, 1);
    }

    @Override
    public double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;

        return angle;
    }

    @Override
    public double headingFromRelativePosition(double x, double y) {
        double h = Math.atan2(x, y)*(180/Math.PI)-90;
        return normalizeAngle(h);
    }

    @Override
    public void turnTo(double turnSpeed, double heading) {
        throw new UnsupportedOperationException();
    }

    private void _moveRobot(double drive, double turn) {
        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        lf.setPower(leftSpeed);
        rf.setPower(rightSpeed);
        lb.setPower(leftSpeed);
        rb.setPower(rightSpeed);

    }
    private void _strafeRobot(double leftSpeed, double rightSpeed, double leftBackSpeed, double rightBackSpeed, double turn) {
        leftSpeed -= turn;
        rightSpeed += turn;
        leftBackSpeed -= turn;
        rightBackSpeed += turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
            leftBackSpeed /= max;
            rightBackSpeed /= max;
        }

        lf.setPower(leftSpeed);
        rf.setPower(rightSpeed);
        lb.setPower(leftBackSpeed);
        rb.setPower(rightBackSpeed);

    }

    @Override
    public void stop() {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    @Override
    public void coast() {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    @Override
    public void startDrive(double speed, double direction, double turnSpeed) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turn = _getSteeringCorrection(direction, turnSpeed/50);

        _strafeRobot(speed, speed, speed, speed, turn);
    }

    @Override
    public void startStrafe(double speed, double direction, double turnSpeed) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double axial   = Math.sin(Math.PI/180*direction);
        double lateral = Math.cos(Math.PI/180*direction);
        double turn = turnSpeed*P_TURN_GAIN;

        double leftFrontPower  = (axial + lateral + turn);
        double rightFrontPower = (axial - lateral - turn);
        double leftBackPower   = (axial - lateral + turn);
        double rightBackPower  = (axial + lateral - turn);
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontPower *= speed;
        rightFrontPower *= speed;
        leftBackPower *= speed;
        rightBackPower *= speed;

        // double turn = _getSteeringCorrection(getHeading()+deltaHeading, turnSpeed/10);

        //double turn = _getSteeringCorrection(heading, turnSpeed/50); <- Old version ^^^

        _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);
    }

    @Override
    public void startStrafe(double speed, double direction) {
        startStrafe(speed, direction, 0);
    }

    @Override
    public void startTurn(double turnSpeed) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        lf.setPower(turnSpeed);
        lb.setPower(turnSpeed);
        rf.setPower(-turnSpeed);
        rb.setPower(-turnSpeed);
    }
}

