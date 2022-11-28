package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Function;

public class RobotSpecifications {
    // DRIVE STUFF
    public static final int driveMotorCpr = 24; // cpr
    public static final int driveMotorGearRatio = 20/1; // gear ratio a fraction
    public static final int driveWheelCpr = driveMotorCpr*driveMotorGearRatio; // cpr
    public static final int driveWheelRadius = 49; // mm
    public static final double driveWheelCountsPerMm = driveWheelCpr /(driveWheelRadius*2*Math.PI); // counts per mm
    public static final double driveMotorRadiansPerDriveWheelMm = driveMotorGearRatio/(double)driveWheelRadius;

    public static final RevHubOrientationOnRobot revHubOrientationOnRobot =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    // TODO measure these
    public static final double driveBaseRadius = 239; // distance from center of wheel to center of robot; mm
    public static final double robotLength = 406.4; // mm
    public static final double robotWidth = 254; // mm
    public static final double robotHeight = 100; // mm

    // ARM STUFF
    public static final double armRadius = 350; // distance from axle to axle; mm
    public static final double armMotorCpr = 24; // cpr
    public static final double armMotorGearRatio = 120/1; // gear ratio as a fraction
    public static final double lostAngle = AngleUnit.DEGREES.toRadians(10); // "lost angle" (arm not resting vertical) so some theta is lost; radians
    public static final double armCpr = armMotorCpr*armMotorGearRatio; // cpr
    public static final Function<Integer, Double> armTicsToRadians = (Integer tics) -> tics/armCpr*2*Math.PI+lostAngle;

}
