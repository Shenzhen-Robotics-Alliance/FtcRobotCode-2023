package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class AutoStageChassisModule {
    private final HardwareDriver driver;
    private final IMU imu;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;

    private double[] robotCurrentPosition = new double[2];
    private double robotCurrentRotation; // store the status of the robot, in reference to the starting point, which is the left side of the field, in mm

    public AutoStageChassisModule(HardwareDriver driver, HardwareMap hardwareMap) {
        this.driver = driver;
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
    }
}
