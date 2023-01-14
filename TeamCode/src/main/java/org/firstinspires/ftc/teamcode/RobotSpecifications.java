package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Point;

import java.util.function.Function;

public class RobotSpecifications {
    public static final int TORQUENADO_CPR = 24;
    // DRIVE STUFF
    public static final int driveMotorCpr = TORQUENADO_CPR; // cpr
    public static final double driveMotorGearRatio = 20/1; // gear ratio a fraction
    public static final int driveWheelCpr = (int) (driveMotorCpr*driveMotorGearRatio); // cpr
    public static final int driveWheelRadius = 49; // mm
    public static final double driveWheelCountsPerMm = driveWheelCpr /(driveWheelRadius*2*Math.PI); // counts per mm
    public static final double driveMotorRadiansPerDriveWheelMm = driveMotorGearRatio/(double)driveWheelRadius;

    // TODO MEASURE AND TUNE
    public static final WheelParameters frontLeftParameters = new WheelParameters(Math.sqrt(2)/2, -Math.sqrt(2)/2, new Point(-1,1, DistanceUnit.MM));
    public static final WheelParameters frontRightParameters = new WheelParameters(Math.sqrt(2)/2, Math.sqrt(2)/2, new Point(1,1, DistanceUnit.MM));
    public static final WheelParameters rearLeftParameters = new WheelParameters(Math.sqrt(2)/2, Math.sqrt(2)/2, new Point(-1,-1, DistanceUnit.MM));
    public static final WheelParameters rearRightParameters = new WheelParameters(Math.sqrt(2)/2, -Math.sqrt(2)/2, new Point(1,-1, DistanceUnit.MM));

    public static final RevHubOrientationOnRobot revHubOrientationOnRobot =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.DOWN);

    // TODO measure these
    public static final double driveBaseRadius = 239; // distance from center of wheel to center of robot; mm
//    public static final double robotLength = 406.4; // mm
//    public static final double robotWidth = 254; // mm
//    public static final double robotHeight = 100; // mm

    // ARM STUFF TODO MEASURE THESE
    public static final double armRadius = 550; // distance from axle to axle; mm
    public static final int armMotorCpr = TORQUENADO_CPR; // cpr
    public static final double armMotorGearRatio = 30/1; // gear ratio as a fraction (including gearbox)
    public static final double lostAngle = AngleUnit.DEGREES.toRadians(5); // "lost angle" (arm not resting vertical) so some theta is lost; radians
    public static final double armCpr = armMotorCpr*armMotorGearRatio; // cpr
    public static final double armClawDistance = 45; // distance from bottom of claw to axle; mm
    public static final double clawDistanceAboveGround = 44; // mm
    public static final double towerHeightToAxle = 437; // mm
    public static final double towerHeightAboveGround = 96; //mm
    public static final Function<Integer, Double> armTicsToRadians = (Integer tics) -> tics/armCpr*2*Math.PI+lostAngle;

}
