package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

    public void initRobotChassis() {
        final double imuXRotation = 0;
        final double imuYRotation = 145.64;
        final double imuZRotation = 0;// init the imu
        Orientation imuRotation = xyzOrientation(imuXRotation, imuYRotation, imuZRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(imuRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void setRobotRotation(double targetedRotation) { // rote the robot to targeted spot, in radian
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        final double imuRotationCorrectionRate = -1;
        double currentRotation = orientation.getYaw(AngleUnit.RADIANS) * imuRotationCorrectionRate;
        if (currentRotation < 0) currentRotation = 2*Math.PI + currentRotation;

        double rotationDifference = targetedRotation - currentRotation;
        while (rotationDifference > 0) {
            double expectedRotationalVelocity = new ChassisModule(null, null, imu).linearMap(
                    0, (Math.PI / 2), 0.6, 1, rotationDifference
            ); // the robot spins in a velocity that depends on the rotation difference
            setRobotMotion(0, 0, expectedRotationalVelocity);
        }
    }

    private void setRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        driver.leftFront.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
        driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        driver.rightRear.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
    }
}
