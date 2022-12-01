package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

public final class Point {
    private final double x;
    private final double y;
    private final DistanceUnit distanceUnit;

    public Point(double x, double y, DistanceUnit distanceUnit) {
        this.x = x;
        this.y = y;
        this.distanceUnit = distanceUnit;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public DistanceUnit distanceUnit() {
        return distanceUnit;
    }

    public Point toDistanceUnit(DistanceUnit newUnits) {
        return new Point(newUnits.fromUnit(distanceUnit, x), newUnits.fromUnit(distanceUnit, y), newUnits);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (obj == null || obj.getClass() != this.getClass()) return false;
        Point that = (Point) obj;
        return Double.doubleToLongBits(this.x) == Double.doubleToLongBits(that.x) &&
                Double.doubleToLongBits(this.y) == Double.doubleToLongBits(that.y) &&
                Objects.equals(this.distanceUnit, that.distanceUnit);
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, distanceUnit);
    }

    @Override
    public String toString() {
        return "Point[" +
                "x=" + x + ", " +
                "y=" + y + ", " +
                "distanceUnit=" + distanceUnit + ']';
    }

}
