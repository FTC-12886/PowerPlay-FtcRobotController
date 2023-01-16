package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.file.MapItem;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;

@Autonomous()

public class DriveForwards extends LinearOpMode {
    private String[] presetName = {"Short (36 in)","Long (90 in)"};
    private Double[] presetDistance = {36.0,90.0};
    private int i = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        telemetry.log().add("Set the distance with the dpad left and right. Point the robot in the correct direction and press PLAY.");
        telemetry.update();
        while (opModeInInit()) {
            telemetry.addData("Preset", presetName[i]);
            telemetry.update();

            if (gamepad1.dpad_left) {
                if (i > 0) // if not at start of array
                    i--;
                else // loop back if at end of array
                    i=presetName.length-1;
                sleep(50);
            } else if (gamepad1.dpad_right) {
                if (i < presetName.length-1) // if not at end of array
                    i++;
                else // loop back if at end of array
                    i=0;
                sleep(50);
            }
        }

        waitForStart();
        robot.drive(0.25, DistanceUnit.METER);
        while (robot.getYPosition(DistanceUnit.INCH) < presetDistance[i]) sleep(5);
        robot.stop();
        requestOpModeStop();
    }

}
