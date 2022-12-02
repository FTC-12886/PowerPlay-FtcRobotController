package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Point;

@TeleOp
public class AbstractedTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry.addData("Status", "Ready, press start to continue");
        telemetry.update();
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x);
        double leftStickY = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double rightStickX = 2*gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        double rightStickY = -gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y);

        double theta = Math.atan(leftStickY/leftStickX);
        if (leftStickX<0 && leftStickY<0) // tan also negative in quad 3
            theta += Math.PI;
        else if (leftStickX<0)
            theta = Math.PI+theta;

        robot.drive(Math.hypot(leftStickX, leftStickY), DistanceUnit.METER, theta, AngleUnit.RADIANS, rightStickX, AngleUnit.RADIANS);
        Point position = robot.getPosition(DistanceUnit.METER);
        if (gamepad1.dpad_up) {
            robot.setArmPosition(robot.getArmPosition() + 20);
        } else if (gamepad1.dpad_down) {
            robot.setArmPosition(robot.getArmPosition() - 20);
        }

        if (gamepad1.dpad_left) {
            robot.setClawPosition(robot.getClawPosition()-.05);
        } else if (gamepad1.dpad_right) {
            robot.setClawPosition(robot.getClawPosition()+.05);
        }

        telemetry.addData("x", position.x());
        telemetry.addData("y", position.y());
        telemetry.addData("arm", robot.getArmPosition());
    }
}
