package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ScorpChassis implements RobotChassis {
    private       DcMotor      lf;
    private       DcMotor      rf;
    private       DcMotor      lb;
    private       DcMotor      rb;
    private       SparkFunOTOS otos;
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
    static final double     HEADING_THRESHOLD       = 2.0;
    static final double     P_TURN_GAIN             = 0.05;
    static final double     P_DRIVE_GAIN            = 0.03;
    static final double     ACCURACY                = 2.0;
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

    //High level - Simple
    public void moveTo(double x, double y, double driveSpeed){
        if (lf == null || lb == null || rf == null || rb == null || otos == null) {
            return;
        }
        double fh = getHeading(); // initial and final heading
        SparkFunOTOS.Pose2D pos = getPosition();
        double dx = x - pos.x;
        double dy = y - pos.y;
        double distance = Math.sqrt(dx*dx+dy*dy);
        double h = 0;

        turnToHeading(DRIVE_SPEED_SLOW, headingFromRelativePosition(dx, dy));
        while(Math.abs(distance) > ScorpChassis.ACCURACY && op.opModeIsActive()) {
            // lower driveSpeed as we get closer, and return to desired heading
            if (distance < 3) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_SLOW/2);
                h = fh;
            } else if (distance < 5) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_SLOW);
                h = fh;
            } else if (distance < 10) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_NORMAL);
                h = fh;
            } else {
                // try to face the target
                h = headingFromRelativePosition(dx, dy);
            }

            _startDriveStraight(driveSpeed, h);

            if (ScorpChassis.DEBUG) {
                this.op.telemetry.addLine("moveTo");
                this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
                this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
                this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
                this.op.telemetry.addData("Distance:", "%.4f",distance);
                this.op.telemetry.addData("Desired heading:", "%.4f", h);
                this.op.telemetry.addData("Current heading:", "%.4f", pos.h);
                this.op.telemetry.addData("DriveSpeed:", "%.4f", driveSpeed);
                this.op.telemetry.update();
            }

            pos = getPosition();
            dx = x - pos.x;
            dy = y - pos.y;
            distance = Math.sqrt(dx*dx+dy*dy);
        }
        stop();
        if (ScorpChassis.DEBUG ) {
            this.op.telemetry.addLine("moveTo finished");
            this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
            this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
            this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
            this.op.telemetry.addData("Desired heading:", "%.4f", h);
            this.op.telemetry.addData("Current heading:", "%.4f", pos.h);
            this.op.telemetry.update();
        }
    }


    public void strafeTo(double x, double y, double driveSpeed){
        strafeTo(x, y, driveSpeed, getHeading());
    }
    //High level - Simple
    public void strafeTo(double x, double y, double driveSpeed, double heading){
        if (lf == null || lb == null || rf == null || rb == null || otos == null) {
            return;
        }
        SparkFunOTOS.Pose2D pos = getPosition(); // Get the robots starting position
        double dx = x - pos.x; // Subtract the target x by starting x
        double dy = y - pos.y; // Subtract target y by starting y
        double distance = Math.sqrt(dx*dx+dy*dy); // Calculate the starting distance between target
        double direction  = headingFromRelativePosition(dx, dy); // gets a direction towards target

        while(Math.abs(distance) > ScorpChassis.ACCURACY && op.opModeIsActive()) { // runs unless we are within two inches of target or the program has deactivated
            // lower driveSpeed as we get closer, and return to desired heading
            if (distance < 3) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_SLOW/2);
            } else if (distance < 5) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_SLOW);
            } else if (distance < 10) {
                driveSpeed = Math.min(driveSpeed, ScorpChassis.DRIVE_SPEED_NORMAL);
            }

            startStrafe(driveSpeed, direction); // Provides direction and speed so we start moving

            // Telemetry
            if (ScorpChassis.DEBUG) {
                this.op.telemetry.addLine("strafeTo");
                this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
                this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
                this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
                this.op.telemetry.addData("Distance:", "%.4f",distance);
                this.op.telemetry.addData("Direction:", "%.4f", direction);
                this.op.telemetry.addData("DriveSpeed:", "%.4f", driveSpeed);
                this.op.telemetry.update();
            }

            pos = getPosition(); // Gets new position
            dx = x - pos.x; // Gets new dx
            dy = y - pos.y; // Gets new dy
            distance = Math.sqrt(dx*dx+dy*dy); // Gets new distance
            direction  = headingFromRelativePosition(dx, dy); // Gets new direction
        }
        stop(); // Stops all movement when we are within two inches of target
        turnToHeading(DRIVE_SPEED_SLOW, heading); // Turns to the desired end heading
        // Telemetry
        if (ScorpChassis.DEBUG ) {
            this.op.telemetry.addLine("strafeTo finished");
            this.op.telemetry.addData("Target:", "%.4f, %.4f", x, y);
            this.op.telemetry.addData("Position:", "%.4f, %.4f", pos.x, pos.y);
            this.op.telemetry.addData("Delta:", "%.4f, %.4f", dx, dy);
            this.op.telemetry.addData("Direction:", "%.4f", direction);
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

            this.op.telemetry.addLine("turnToHeading");
            this.op.telemetry.addData("turnSpeed:", "%.4f", turnSpeed);
            this.op.telemetry.addData("Current heading:", "%.4f", current);
            this.op.telemetry.addData("Heading error:", "%.4f", headingError);
            this.op.telemetry.update();
        }
        stop();
    }

    private void _startDriveStraight(double maxDriveSpeed, double h) {
        if (lf == null || lb == null || rf == null || rb == null ) {
            return;
        }
        double turnSpeed = _getSteeringCorrection(h, P_TURN_GAIN);
        if (ScorpChassis.DEBUG) {
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
        if (ScorpChassis.DEBUG) {
            this.op.telemetry.addData("sds maxDriveSpeed:", "%.4f", maxDriveSpeed);
            this.op.telemetry.addData("sds turnSpeed: ", "%.4f", turnSpeed);
        }
        _moveRobot(maxDriveSpeed, turnSpeed);
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

         return Range.clip(headingError * proportionalGain, -0.3, 0.3);
    }

    @Override
    public double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;

        return angle;
    }

    @Override
    public double headingFromRelativePosition(double x, double y) {
        double h = Math.atan2(y, x)*(180/Math.PI)-90;
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
        //op.telemetry.addData("sp", "%.2f", speed);
        //op.telemetry.addData("dr", "%.2f", direction);
        //op.telemetry.addData("ts", "%.2f", turnSpeed);
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

        op.telemetry.addData("LF", "%.2f", leftFrontPower);
        op.telemetry.addData("RF", "%.2f", rightFrontPower);
        op.telemetry.addData("LB", "%.2f", leftBackPower);
        op.telemetry.addData("RB", "%.2f", rightBackPower);

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

