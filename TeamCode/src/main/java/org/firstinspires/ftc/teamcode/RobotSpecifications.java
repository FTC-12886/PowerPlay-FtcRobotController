package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotSpecifications {
    public static final int driveMotorCpr = 24; // cpr
    public static final int driveMotorGearRatio = 20/1; // a fraction
    public static final int driveWheelCpr = driveMotorCpr*driveMotorGearRatio; // cpr
    public static final int driveWheelRadius = 49; // mm
    public static final int driveWheelCountsPerMm = (int) Math.round(driveWheelCpr /(driveWheelRadius*2*Math.PI));

    public static final RevHubOrientationOnRobot revHubOrientationOnRobot =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    // TODO measure these
    public static final double robotLength = 406.4; // mm
    public static final double robotWidth = 254; // mm
    public static final double robotHeight = 100; // mm

}
