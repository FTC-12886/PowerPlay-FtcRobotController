package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class PrintArmEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("tics", robot.getArmPosition());
            telemetry.addData("theta", robot.getArmAngle(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
