package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Point;

@TeleOp
@Config
public class AbstractedTeleOp extends OpMode {
    private Robot robot;

    private double mmAboveGround = RobotSpecifications.clawDistanceAboveGround+RobotSpecifications.armClawDistance;
    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double lsx = 1;
    public static double lsy = 1;
    public static double rsx = 1.5;
    public static double pow = 1;

    public static int a = 0;
    public static int b = 100;
    public static int x = 300;
    public static int y = 500;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry.addData("Status", "Ready, press start to continue");
        telemetry.update();
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        double leftStickX = (gamepad1.right_bumper ? 2 : 1)*lsx*gamepad1.left_stick_x*Math.pow(Math.abs(gamepad1.left_stick_x), pow);
        double leftStickY = (gamepad1.right_bumper ? 2 : 1)*-lsy*gamepad1.left_stick_y*Math.pow(Math.abs(gamepad1.left_stick_y),pow);
        double rightStickX = rsx*gamepad1.right_stick_x*Math.pow(Math.abs(gamepad1.right_stick_x),pow);
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

        telemetry.addData("time", time.time());

        if (gamepad1.a) {
            robot.setArmPosition(a);
        } else if (gamepad1.b) {
            robot.setArmPosition(b);
        } else if (gamepad1.x) {
            robot.setArmPosition(x);
        } else if (gamepad1.y) {
            robot.setArmPosition(y);
        }

        if (gamepad1.left_trigger > 0.75) {
            robot.openClaw();
        } else if (gamepad1.right_trigger > 0.75) {
            robot.closeClaw();
        }

        robot.loop();
        time.reset();
        telemetry.addData("x", position.x());
        telemetry.addData("y", position.y());
        telemetry.addData("arm", robot.getArmPosition());
//        telemetry.addData("servo", robot.getClawPosition());
    }
}
