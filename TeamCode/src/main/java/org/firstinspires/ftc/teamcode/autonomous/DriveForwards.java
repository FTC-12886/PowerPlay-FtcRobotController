package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous()
public class DriveForwards extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        telemetry.addData(">","The robot will drive forwards about 1 tile length. Point the robot in the correct direction and press PLAY.");
        telemetry.update();

        waitForStart();
        robot.drive(0.25, DistanceUnit.METER);
        while (robot.getYPosition(DistanceUnit.METER) < 1.5) sleep(5);
        robot.stop();
        requestOpModeStop();
    }

}
