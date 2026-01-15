package org.firstinspires.ftc.teamcode.chassis;
import static org.firstinspires.ftc.teamcode.util.Position.headingFromRelativePosition;
import static org.firstinspires.ftc.teamcode.util.Position.normalizeAngle;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

public class ScorpChassisOdometry extends ScorpChassisBase {
    protected DcMotor lf = null;
    protected DcMotor rf = null;
    protected DcMotor lb = null;
    protected DcMotor rb = null;
    protected IMU imu = null;
    protected LinearOpMode op = null;
    public ScorpCamera camera;

    @Override
    public Position getPosition() {
        return new Position(0,0, getIMUHeading());
    }
    public void resetPositionAndHeading() {
        imu.resetYaw();
    }

    public ScorpChassisOdometry(LinearOpMode op, String lfName, String rfName, String lbName, String rbName, String imuName) {
        try {
            this.lf = op.hardwareMap.get(DcMotor.class, lfName);
            this.rf = op.hardwareMap.get(DcMotor.class, rfName);
            this.lb = op.hardwareMap.get(DcMotor.class, lbName);
            this.rb = op.hardwareMap.get(DcMotor.class, rbName);
            this.imu = op.hardwareMap.get(IMU.class, imuName);
            this.op = op;
            this.camera = new ScorpCamera(op, "Webcam 1");
        } catch (Exception ignored) {
        }
    }
    public void init(){
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
    }
    public void autoAim(){
//        List<Integer> tags = Arrays.asList(24, 25);
//        AprilTagDetection tag;
//        int id = 0;
//        tag = camera.getDetection(tags);
//        if (tag != null) {
//            id = tag.id;
//        }

        List<Integer> tags = Arrays.asList(24, 20); // red - 24, blue - 20
        AprilTagDetection tag;
        tag = camera.getDetection(tags);
        if(tag != null){

        }
    }
    //void moveTo(double x, double y, double driveSpeed);
    public void strafeTo(double x, double y, double driveSpeed){
        strafeTo(x, y, driveSpeed, 0);
    }
    public void strafeTo(double x, double y, double driveSpeed, double heading){
        if (lf == null || lb == null || rf == null || rb == null) {
            return;
        }
        // new: This resets us before strafe.
        // x, y are relative to current heading and position
        this.resetPositionAndHeading();

        heading = normalizeAngle(heading);
        Position pos = getPosition();
        double dx = x - pos.x;
        double dy = y - pos.y;
        double distance = Math.sqrt(dx*dx+dy*dy);
        double direction  = headingFromRelativePosition(dx, dy);
        while(Math.abs(distance) > ScorpChassisOdometry.ACCURACY && op.opModeIsActive()) {
            if (distance < 5) {
                driveSpeed = Math.min(driveSpeed, ScorpChassisOdometry.DRIVE_SPEED_SLOW);
            } else if (distance < 20) {
                driveSpeed = Math.min(driveSpeed, ScorpChassisOdometry.DRIVE_SPEED_NORMAL);
            }
            startStrafeAbsolute(driveSpeed, direction, heading);
            if (ScorpChassisOdometry.DEBUG) {
                this.op.telemetry.addLine("strafeTo");
                this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
                this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
                this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
                this.op.telemetry.addData("Distance:", "%.4f",distance);
                this.op.telemetry.addData("Direction:", "%.4f", direction);
                this.op.telemetry.addData("DriveSpeed:", "%.4f", driveSpeed);
                this.op.telemetry.update();
            }
            pos = getPosition();
            dx = x - pos.x;
            dy = y - pos.y;
            distance = Math.sqrt(dx*dx+dy*dy);
            direction  = headingFromRelativePosition(dx, dy);
        }
        stopDrive();
        if (ScorpChassisOdometry.DEBUG ) {
            this.op.telemetry.addLine("strafeTo finished");
            this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
            this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
            this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
            this.op.telemetry.addData("Direction:", "%.4f", direction);
            this.op.telemetry.update();
        }
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }

        // new: this makes heading relative to current heading
        resetPositionAndHeading();
//        this.op.telemetry.addLine("++++++++++++++++++++ start of: turnToHeading");
//        this.op.telemetry.update();
//        this.op.sleep(3000);
        heading = normalizeAngle(heading);
        double turnSpeed;
        //_getSteeringCorrection(heading, P_DRIVE_GAIN);
        double current = getIMUHeading();
        double headingError = normalizeAngle(heading - current);
        while (op.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = _getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            if (Math.abs(headingError) < 5) {
                turnSpeed = Range.clip(turnSpeed, -0.1, 0.1);
            } else if (Math.abs(headingError) < 15) {
                turnSpeed = Range.clip(turnSpeed, -0.3, 0.3);
            }
            _moveRobot(0, turnSpeed);
            current = getIMUHeading();
            headingError = normalizeAngle(heading - current);

//            this.op.telemetry.addLine("++++++++++++++++++++ turnToHeading");
//            this.op.telemetry.addData("turnSpeed:", "%.4f", turnSpeed);
//            this.op.telemetry.addData("Current heading:", "%.4f", current);
//            this.op.telemetry.addData("Heading error:", "%.4f", headingError);
//            this.op.telemetry.update();
        }
//        this.op.sleep(3000);
//        this.op.telemetry.addLine("xxxxxxxxxxxxxxxxxxxxx done with turn code, before stop");
//        this.op.telemetry.update();
        stopDrive();
//        this.op.telemetry.addLine("yyyyyyyyyyyyyyyyyyyyy done with turn code, after stop");
//        this.op.telemetry.update();
    }
    private void _startDriveStraight(double maxDriveSpeed, double h) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turnSpeed = _getSteeringCorrection(h, P_TURN_GAIN);
        if (ScorpChassisOdometry.DEBUG) {
            this.op.telemetry.addData("sds maxDriveSpeed:", "%.4f", maxDriveSpeed);
            this.op.telemetry.addData("sds turnSpeed: ", "%.4f", turnSpeed);
        }
        _moveRobot(Math.abs(maxDriveSpeed), turnSpeed);
    }
    private void _startDriveStraightOrBack(double maxDriveSpeed, double h) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turnSpeed = _getSteeringCorrection(h, P_TURN_GAIN);
        if (ScorpChassisOdometry.DEBUG) {
            this.op.telemetry.addData("sds maxDriveSpeed:", "%.4f", maxDriveSpeed);
            this.op.telemetry.addData("sds turnSpeed: ", "%.4f", turnSpeed);
        }
        _moveRobot(maxDriveSpeed, turnSpeed);
    }

    // can be overriden by subclass
    public double getIMUHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public String getPositionString() {
        Position p = getPosition();
        return "h: "+p.h+", x: "+p.x+", y:"+p.y;
    }

    protected double _getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = getIMUHeading() - desiredHeading;

        headingError = normalizeAngle(headingError);
        op.telemetry.addData("he", "%.2f", headingError);

         return Range.clip(-headingError * proportionalGain, -0.3, 0.3);
    }

    public void turnTo(double turnSpeed, double heading) {
        throw new UnsupportedOperationException();
    }

    protected void _moveRobot(double drive, double turn) {
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

    protected void _strafeRobot(double leftSpeed, double rightSpeed, double leftBackSpeed, double rightBackSpeed, double turn) {
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


    public void stopDrive() {
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


    public void startDrive(double speed, double direction, double turnSpeed) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turn = _getSteeringCorrection(direction, turnSpeed/50);

        _strafeRobot(speed, speed, speed, speed, turn);
    }

    public void startStrafeAbsolute(double speed, double direction) {
        this.startStrafeAbsolute(speed, direction, this.getIMUHeading());
    }

    public void startStrafeAbsolute(double speed, double direction, double heading) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double axial   = Math.cos(Math.PI/180*(direction- getIMUHeading()));
        double lateral = Math.sin(Math.PI/180*(direction- getIMUHeading()));
        double turn = _getSteeringCorrection(heading, P_TURN_GAIN);
        op.telemetry.addData("hd", "%.2f", getIMUHeading());
        op.telemetry.addData("dh", "%.2f", heading);
        op.telemetry.addData("ax", "%.2f", axial);
        op.telemetry.addData("lt", "%.2f", lateral);
        op.telemetry.addData("tn", "%.2f", turn);

        double leftFrontPower  = (axial - lateral - turn);
        double rightFrontPower = (axial + lateral + turn);
        double leftBackPower   = (axial + lateral - turn);
        double rightBackPower  = (axial - lateral + turn);
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

        op.telemetry.addData("LF", "%.2f", leftFrontPower);
        op.telemetry.addData("RF", "%.2f", rightFrontPower);
        op.telemetry.addData("LB", "%.2f", leftBackPower);
        op.telemetry.addData("RB", "%.2f", rightBackPower);

        _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);
    }

    @Override
    public void startStrafe(double speed, double direction, double turn) {
        startStrafe(speed, direction, turn, getIMUHeading());
    }

    @Override
    public void startStrafe(double speed, double direction) {
        startStrafe(speed, direction, 0);
    }

    @Override
    public void startStrafe(double speed, double direction, double turn, double heading) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        op.telemetry.addData("sp", "%.2f", speed);
        op.telemetry.addData("dr", "%.2f", direction);
        op.telemetry.addData("ts", "%.2f", turn);
        double axial   = Math.cos(Math.PI/180*(direction));
        double lateral = Math.sin(Math.PI/180*(direction));
        if (turn == 0) {
            turn = _getSteeringCorrection(heading, P_TURN_GAIN)*Math.abs(lateral);
        }
        op.telemetry.addData("tn", "%.2f", turn);

        double leftFrontPower  = (axial - lateral - turn);
        double rightFrontPower = (axial + lateral + turn);
        double leftBackPower   = (axial + lateral - turn);
        double rightBackPower  = (axial - lateral + turn);
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

        op.telemetry.addData("LF", "%.2f", leftFrontPower);
        op.telemetry.addData("RF", "%.2f", rightFrontPower);
        op.telemetry.addData("LB", "%.2f", leftBackPower);
        op.telemetry.addData("RB", "%.2f", rightBackPower);

        // double turn = _getSteeringCorrection(getHeading()+deltaHeading, turnSpeed/10);

        //double turn = _getSteeringCorrection(heading, turnSpeed/50); <- Old version ^^^

        _strafeRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, 0);
    }

    public void startTurn(double turnSpeed) {
        if (lf == null || lb == null || rf == null || rb == null) {
            return;
        }
        lf.setPower(-turnSpeed);
        lb.setPower(-turnSpeed);
        rf.setPower(+turnSpeed);
        rb.setPower(+turnSpeed);
    }
}

