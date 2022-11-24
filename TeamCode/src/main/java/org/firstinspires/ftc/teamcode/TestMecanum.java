package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

@TeleOp
public class TestMecanum extends OpMode {

    private DcMotorEx rearRight;
    private DcMotorEx rearLeft;
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private BNO055IMU imu;
    private boolean motorsDisabled = false;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearRight = hardwareMap.get(DcMotorEx.class, "backRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addData("Status", "Ready, press start to continue");
        telemetry.addData("","Press A to disable motors");
        telemetry.update();

//        hardwareMap.iterator().next().getClass();
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x); // quadratic curve but flipped
        double leftStickY = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double rightStickX = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        double rightStickY = -gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y);
        double rearRightPower = 0;
        double rearLeftPower = 0;
        double frontRightPower = 0;
        double frontLeftPower = 0;

        if (gamepad1.a) {
            motorsDisabled = !motorsDisabled;
//            try {
//                wait(50); // debounce
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
        }

        // mecanum code

        // forwards/backwards
        rearRightPower += leftStickY;
        rearLeftPower += leftStickY;
        frontRightPower += leftStickY;
        frontLeftPower += leftStickY;

        // sideways
        rearRightPower -= leftStickX;
        rearLeftPower += leftStickX;
        frontRightPower += leftStickX;
        frontLeftPower -= leftStickX;

        // turning
        frontLeftPower += rightStickX;
        rearLeftPower += rightStickX;

        frontRightPower -= rightStickX;
        rearRightPower -= rightStickX;

//        telemetry.addData("FL", frontLeftPower);
//        telemetry.addData("FR", frontRightPower);
//        telemetry.addData("RL", rearLeftPower);
//        telemetry.addData("RR", rearRightPower);
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("RL", rearLeft.getCurrentPosition());
        telemetry.addData("RR", rearRight.getCurrentPosition());

        telemetry.addData("FL mm", frontLeft.getCurrentPosition()/RobotSpecifications.driveWheelCountsPerMm);
        telemetry.addData("FR mm", frontRight.getCurrentPosition()/RobotSpecifications.driveWheelCountsPerMm);
        telemetry.addData("RL mm", rearLeft.getCurrentPosition()/RobotSpecifications.driveWheelCountsPerMm);
        telemetry.addData("RR mm", rearRight.getCurrentPosition()/RobotSpecifications.driveWheelCountsPerMm);

        telemetry.addData("x", orientation.firstAngle);
        telemetry.addData("y", orientation.secondAngle);
        telemetry.addData("z", orientation.thirdAngle);
        telemetry.addData("Motors Disabled", motorsDisabled);
        if (motorsDisabled) {
            rearRight.setMotorDisable();
            rearLeft.setMotorDisable();
            frontLeft.setMotorDisable();
            frontRight.setMotorDisable();
        } else {
            rearRight.setMotorEnable();
            rearLeft.setMotorEnable();
            frontLeft.setMotorEnable();
            frontRight.setMotorEnable();
        }

        rearRight.setPower((gamepad1.left_bumper ? 1 : 0.5)*rearRightPower);
        rearLeft.setPower((gamepad1.left_bumper ? 1 : 0.5)*rearLeftPower);
        frontRight.setPower((gamepad1.left_bumper ? 1 : 0.5)*frontRightPower);
        frontLeft.setPower((gamepad1.left_bumper ? 1 : 0.5)*frontLeftPower);
    }
}
