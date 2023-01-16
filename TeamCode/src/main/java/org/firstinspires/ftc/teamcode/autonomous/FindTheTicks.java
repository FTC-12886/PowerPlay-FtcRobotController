package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class FindTheTicks extends OpMode {
    Robot robot;
    int pos = 0;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        robot.setArmPosition(pos);
        double amps = robot.getArmLift().getCurrent(CurrentUnit.AMPS);
        telemetry.addData("amps",amps );
        telemetry.addData("pos",pos );
        if (gamepad1.dpad_up) {
            pos+=5;
        }else if (gamepad1.dpad_down) {
            pos-=5;
        }
    }
}
