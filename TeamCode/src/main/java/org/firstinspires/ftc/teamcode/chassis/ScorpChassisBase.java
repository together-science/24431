package org.firstinspires.ftc.teamcode.chassis;

import org.firstinspires.ftc.teamcode.util.Position;

public abstract class ScorpChassisBase {
    public static final double DRIVE_SPEED_FAST = 0.4;
    public static final double DRIVE_SPEED_NORMAL = 0.3;
    public static final double DRIVE_SPEED_SLOW = 0.2;
    static final double HEADING_THRESHOLD = 2.0;
    static final double P_TURN_GAIN = 0.05;
    static final double P_DRIVE_GAIN = 0.03;
    static final double ACCURACY = 2.0;
    static final boolean DEBUG = true;

    public abstract void init();
    // nav
    public abstract Position getPosition();
    public abstract double getHeading();
    public abstract String getPositionString();

    // basic movement
    public abstract void stop();
    public abstract void coast();

    // target movement
    public abstract void strafeTo(double x, double y, double driveSpeed);
    public abstract void strafeTo(double x, double y, double driveSpeed, double heading);
    public abstract void turnToHeading(double maxTurnSpeed, double heading);

    // differential movement
    public abstract void startDrive(double speed, double direction, double turnSpeed);
    public abstract void startStrafeAbsolute(double speed, double direction);
    public abstract void startStrafeAbsolute(double speed, double direction, double heading);
    public abstract void startStrafe(double speed, double direction, double turn, double heading);
    public abstract void startStrafe(double speed, double direction, double turn);
    public abstract void startStrafe(double speed, double direction);
    public abstract void startTurn(double turnSpeed);
}

