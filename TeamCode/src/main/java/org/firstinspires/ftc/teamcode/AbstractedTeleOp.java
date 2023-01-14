package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Point;

@TeleOp
public class AbstractedTeleOp extends OpMode {
    private Robot robot;

    private double mmAboveGround = RobotSpecifications.clawDistanceAboveGround+RobotSpecifications.armClawDistance;
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
        else if (leftStickX<0) // quad 2
            theta = Math.PI+theta;

        robot.drive(Math.hypot(leftStickX, leftStickY), DistanceUnit.METER, theta, AngleUnit.RADIANS, rightStickX, AngleUnit.RADIANS);
        Point position = robot.getPosition(DistanceUnit.METER);
        int targetArmPos = robot.getTargetArmPosition();

        robot.setArmPosition((int) (targetArmPos+5*rightStickY));

        telemetry.addData("targetarm", targetArmPos);

        if (gamepad1.a) {
            robot.setArmPosition(0);
        } else if (gamepad1.b) {
            robot.setArmPosition(100);
        } else if (gamepad1.x) {
            robot.setArmPosition(350);
        } else if (gamepad1.y) {
            robot.setArmPosition(650);
        }

        if (gamepad1.left_trigger > 0.75) {
            robot.openClaw();
        } else if (gamepad1.right_trigger > 0.75) {
            robot.closeClaw();
        }

        robot.loop();
        telemetry.addData("x", position.x());
        telemetry.addData("y", position.y());
        telemetry.addData("arm", robot.getArmPosition());
        telemetry.addData("servo", robot.getClawPosition());
    }
}
