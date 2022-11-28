package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        double leftStickX = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x); // quadratic curve but flipped
        double leftStickY = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double rightStickX = 2*gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        double rightStickY = -gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y);

        robot.drive(Math.hypot(leftStickX, leftStickY), DistanceUnit.METER, Math.atan(leftStickY/leftStickX), AngleUnit.RADIANS, rightStickX, AngleUnit.RADIANS);
    }
}
