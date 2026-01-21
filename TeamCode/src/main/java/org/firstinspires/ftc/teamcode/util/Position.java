package org.firstinspires.ftc.teamcode.util;

public class Position {
    public double x, y, h;

    public Position(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public Position() {
    }

    static public double headingFromRelativePosition(double x, double y) {
        // using the pinpoint coordinate system:
        // x positive is forward
        // y positive is left
        // which is rotated from a normal cartesian system by 90 deg counter-clockwise
        // use atan2 like with regular coordinates (x pos right, y pos forward)
        // gives us angle relative to forward
        //
        double h = Math.atan2(y, x)*(180/Math.PI);
        return normalizeAngle(h);
    }

    static public double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;

        return angle;
    }

}