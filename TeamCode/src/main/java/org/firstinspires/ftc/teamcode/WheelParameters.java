package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.Point;

import java.util.Objects;

public final class WheelParameters {
    private final double yPower;
    private final double xPower;
    private final Point positionOnRobot;

    public WheelParameters(double yPower, double xPower, Point positionOnRobot) {
        this.yPower = yPower;
        this.xPower = xPower;
        this.positionOnRobot = positionOnRobot;
    }

    public double yPower() {
        return yPower;
    }

    public double xPower() {
        return xPower;
    }

    public Point positionOnRobot() {
        return positionOnRobot;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (obj == null || obj.getClass() != this.getClass()) return false;
        WheelParameters that = (WheelParameters) obj;
        return Double.doubleToLongBits(this.yPower) == Double.doubleToLongBits(that.yPower) &&
                Double.doubleToLongBits(this.xPower) == Double.doubleToLongBits(that.xPower) &&
                Objects.equals(this.positionOnRobot, that.positionOnRobot);
    }

    @Override
    public int hashCode() {
        return Objects.hash(yPower, xPower, positionOnRobot);
    }

    @Override
    public String toString() {
        return "WheelParameters[" +
                "yPower=" + yPower + ", " +
                "xPower=" + xPower + ", " +
                "positionOnRobot=" + positionOnRobot + ']';
    }

}
