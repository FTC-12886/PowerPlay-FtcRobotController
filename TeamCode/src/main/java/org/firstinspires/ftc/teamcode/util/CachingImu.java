package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class CachingImu implements IMU {

    private final IMU imu;
    private YawPitchRollAngles yawPitchRollAngles;
    private long lastUpdateTime = 0;
    public CachingImu(IMU imu) {
        this.imu = imu;
    }

    public void update() {
        YawPitchRollAngles ypra = imu.getRobotYawPitchRollAngles();
        setYawPitchRollAngles(ypra);
    }

    private synchronized void setYawPitchRollAngles(YawPitchRollAngles yawPitchRollAngles) {
        this.yawPitchRollAngles = yawPitchRollAngles;
        lastUpdateTime = System.nanoTime();
    }
    @Override
    public synchronized boolean initialize(Parameters parameters) {
        return imu.initialize(parameters);
    }

    @Override
    public synchronized void resetYaw() {
        imu.resetYaw();
    }

    @Override
    public synchronized YawPitchRollAngles getRobotYawPitchRollAngles() {
        return yawPitchRollAngles;
    }

    @Override
    public synchronized Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return imu.getRobotOrientation(reference, order, angleUnit);
    }

    @Override
    public synchronized Quaternion getRobotOrientationAsQuaternion() {
        return imu.getRobotOrientationAsQuaternion();
    }

    @Override
    public synchronized AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return imu.getRobotAngularVelocity(angleUnit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return imu.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return imu.getVersion();
    }

    @Override
    public synchronized void resetDeviceConfigurationForOpMode() {
        imu.resetDeviceConfigurationForOpMode();
    }

    @Override
    public synchronized void close() {
        imu.close();
    }
}
