package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotSpecifications;
import org.firstinspires.ftc.teamcode.WheelParameters;
import org.firstinspires.ftc.teamcode.util.Point;

@TeleOp
@Config
public class TimingTest extends OpMode {

    public static double frontLeftX = RobotSpecifications.frontLeftParameters.xPower();
    public static double frontLeftY = RobotSpecifications.frontLeftParameters.yPower();
    public static double frontRightX = RobotSpecifications.frontRightParameters.xPower();
    public static double frontRightY = RobotSpecifications.frontRightParameters.yPower();
    public static double rearLeftX = RobotSpecifications.rearLeftParameters.xPower();
    public static double rearLeftY = RobotSpecifications.rearLeftParameters.yPower();
    public static double rearRightX = RobotSpecifications.rearRightParameters.xPower();
    public static double rearRightY = RobotSpecifications.rearRightParameters.yPower();

    public static int armPos = 30;
    private int oldArmPos = 0;
    public static double speed = 0.5;
    public static double angle = 90;
    private Robot robot;
    @Override
    public void init()  {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        RobotSpecifications.frontLeftParameters = new WheelParameters(frontLeftY, frontLeftX, new Point(-1,1, DistanceUnit.MM));
        RobotSpecifications.frontRightParameters = new WheelParameters(frontRightY, frontRightX, new Point(1,1, DistanceUnit.MM));
        RobotSpecifications.rearLeftParameters = new WheelParameters(rearLeftY, rearLeftX, new Point(-1,-1, DistanceUnit.MM));
        RobotSpecifications.rearRightParameters = new WheelParameters(rearRightY, rearRightX, new Point(1,-1, DistanceUnit.MM));
        if (gamepad1.a) {
            robot.drive(speed, DistanceUnit.METER, angle, AngleUnit.DEGREES);
        } else {
            robot.stop();
        }
        if (armPos != oldArmPos) {
            robot.setArmPosition(armPos);
            oldArmPos = armPos;
        }

    }
}
