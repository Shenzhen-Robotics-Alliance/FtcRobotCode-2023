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
    private IMU imu;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;

    private double[] robotCurrentPosition = new double[2];
    private double robotCurrentRotation; // store the status of the robot, in reference to the starting point, which is the left side of the field, in mm

    public AutoStageChassisModule(HardwareDriver driver, HardwareMap hardwareMap) {
        this.driver = driver;
        this.imu = hardwareMap.get(IMU.class, "imu2"); // use backup imu2 from extension hub if imu does not work
        this.fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
    }

    public void initRobotChassis() {
        final double imuXRotation = 0;
        final double imuYRotation = 145.64;
        final double imuZRotation = 0; // the rotation of the IMU in reference to the vehicle
        Orientation imuRotation = xyzOrientation(imuXRotation, imuYRotation, imuZRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(imuRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void setRobotRotation(double targetedRotation) { // rote the robot to targeted spot, in radian
        final double fullCircle = 2 * Math.PI;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        final double imuRotationCorrectionFactor = -1;
        double currentRotation = orientation.getYaw(AngleUnit.RADIANS) * imuRotationCorrectionFactor;
        if (currentRotation < 0) currentRotation = fullCircle + currentRotation;

        double numericalRotationDifference = targetedRotation - currentRotation;
        double counterClockWiseDifference, clockWiseDifference;

        System.out.print("robot facing:");
        System.out.println(currentRotation);
        // TODO fix bugs on IMU, the IMUs are not working, BNO055 found, but no BHI260AP found, maybe the system went for a wrong IMU
        if (numericalRotationDifference > 0) { // when the target is at the positive(clockwise) direction
            clockWiseDifference = numericalRotationDifference;
            double targetedToOrigin = fullCircle - targetedRotation;
            counterClockWiseDifference = targetedToOrigin + currentRotation;
        } else {
            counterClockWiseDifference = -numericalRotationDifference;
            double targetedToOrigin = fullCircle - targetedRotation;
            clockWiseDifference = targetedToOrigin + currentRotation;
        }

        System.out.println(numericalRotationDifference);

        if (clockWiseDifference < counterClockWiseDifference) {
            rotateClockWise(clockWiseDifference);
        } else {
            rotateCounterClockWise(counterClockWiseDifference);
        }
    }

    private void rotateClockWise(double targetedRotation) {
        double clockWiseDifference ;
        do {
            double currentRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) * -1;
            if (targetedRotation > currentRotation) clockWiseDifference = targetedRotation - currentRotation;
            else clockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of clockwise difference

            double rotatingSpeed = new ChassisModule(null, null, imu)
                    .linearMap(
                            0, Math.toRadians(90), 0.6, 0.85, clockWiseDifference
                    ); // set the speed of rotation depending on the distance left, start to slow down when the difference is smaller than 90deg
            setRobotMotion(0, 0, rotatingSpeed);
            System.out.print("clockwise difference: ");
            System.out.println(clockWiseDifference);
        } while (clockWiseDifference > Math.toRadians(5));
    }

    private void rotateCounterClockWise(double targetedRotation) {
        double counterClockWiseDifference ;
        do {
            double currentRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) * -1;
            if (targetedRotation < currentRotation) counterClockWiseDifference = targetedRotation - currentRotation;
            else counterClockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of counter-clockwise difference

            double rotatingSpeed = new ChassisModule(null, null, imu)
                    .linearMap(
                            0, Math.toRadians(90), 0.6, 0.85, counterClockWiseDifference
                    ) *-1;
            setRobotMotion(0, 0, rotatingSpeed);
            System.out.print("clockwise difference: ");
            System.out.println(counterClockWiseDifference);
        } while (counterClockWiseDifference > Math.toRadians(5));
        setRobotMotion(0, 0, 0);
    }

    private void setRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        driver.leftFront.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
        driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        driver.rightRear.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
    }

    public double getImuYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return -orientation.getYaw(AngleUnit.RADIANS);
    }
}
