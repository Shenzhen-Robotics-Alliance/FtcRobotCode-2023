package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AutoStageChassisModule {
    private final double acceptedRotationDeviation = Math.toRadians(5);
    private final double rotationDifferenceStartDecelerating = Math.toRadians(45);
    private final double minRotatingPower = 0.15;
    private final double stableRotatingPower = 0.45;

    private HardwareDriver driver;
    private final IMUReader imu;
    private Thread imuReaderThread;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;


    private double robotStartingRotation;
    private double[] robotStartingPosition = new double[2];

    private double[] robotCurrentPosition = new double[2];
    private boolean isStopRequested = false;

    public AutoStageChassisModule(HardwareDriver driver, HardwareMap hardwareMap) {
        this.driver = driver;
        this.imu = new IMUReader(hardwareMap); // use backup imu2 from extension hub if imu does not work
        this.fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
    }

    public void initRobotChassis() {
        imu.calibrateIMUHeading();
        this.imuReaderThread = new Thread(() -> {
            while (!isStopRequested) imu.updateIMUStatus();
        });
        imuReaderThread.start();
    }

    public void moveRobotWithEncoder(double targetedXPosition, double targetedYPosition) {
        // move to the requested targeted position, in reference to the starting position

        calculateCurrentEncoderPosition();

        // calculate the required movement to get to the objective position
        double[] requiredMovement = new double[2];
        requiredMovement[0] = targetedXPosition - robotStartingPosition[0]; requiredMovement[1] = targetedYPosition - robotStartingPosition[1];

        // set the targeted position for each motor
        this.driver.leftFront.setTargetPosition((int) (requiredMovement[1] + robotStartingRotation + requiredMovement[0]));
        this.driver.leftFront.setTargetPosition((int) (requiredMovement[1] + robotStartingRotation - requiredMovement[0]));
        this.driver.leftFront.setTargetPosition((int) (requiredMovement[1] - robotStartingRotation - requiredMovement[0]));
        this.driver.leftFront.setTargetPosition((int) (requiredMovement[1] - robotStartingRotation + requiredMovement[0]));

        // set the running parameters for each motors
        
        this.driver.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void calculateCurrentEncoderPosition() {
        this.robotStartingPosition[0] = this.driver.leftFront.getCurrentPosition() + this.driver.rightFront.getCurrentPosition();
        this.robotStartingPosition[1] = this.driver.leftFront.getCurrentPosition() - this.driver.leftRear.getCurrentPosition();
        this.robotStartingRotation = this.driver.leftFront.getCurrentPosition() - this.driver.rightRear.getCurrentPosition();
    }

    public void setRobotRotation(double targetedRotation) { // rote the robot to targeted spot, in radian
        final double fullCircle = 2 * Math.PI;
        double currentRotation = imu.getRobotHeading();
        if (currentRotation < 0) currentRotation = fullCircle + currentRotation;

        double numericalRotationDifference = targetedRotation - currentRotation;
        double counterClockWiseDifference, clockWiseDifference;

        System.out.print("robot facing:");
        System.out.println(currentRotation);
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
            double currentRotation = imu.getRobotHeading();
            if (targetedRotation > currentRotation) clockWiseDifference = targetedRotation - currentRotation;
            else clockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of clockwise difference

            double rotatingSpeed = ChassisModule.linearMap(
                    acceptedRotationDeviation, rotationDifferenceStartDecelerating, minRotatingPower, stableRotatingPower, clockWiseDifference
            ); // set the speed of rotation depending on the distance left, start to slow down when the difference is smaller than 90deg
            setRobotMotion(0, 0, rotatingSpeed);
            // TODO fix bugs on imu data processing
            System.out.print("clockwise difference: ");
            System.out.println(clockWiseDifference);
        } while (clockWiseDifference > Math.toRadians(5));
        setRobotMotion(0, 0, 0);
    }

    private void rotateCounterClockWise(double targetedRotation) {
        double counterClockWiseDifference ;
        do {
            double currentRotation = imu.getRobotHeading();
            if (targetedRotation < currentRotation) counterClockWiseDifference = targetedRotation - currentRotation;
            else counterClockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of counter-clockwise difference

            double rotatingSpeed = ChassisModule.linearMap(
                    acceptedRotationDeviation, rotationDifferenceStartDecelerating, minRotatingPower, stableRotatingPower, counterClockWiseDifference
                    ) *-1;
            setRobotMotion(0, 0, rotatingSpeed);
            System.out.print("counter-clockwise difference: ");
            System.out.println(counterClockWiseDifference);
        } while (counterClockWiseDifference > Math.toRadians(5));
        setRobotMotion(0, 0, 0);
    }

    private void setRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        // control the Mecanum wheel
        driver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        driver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
    }

    public double getImuYaw() {
        return imu.getRobotHeading();
    }

    public void terminate() {
        isStopRequested = true;
    }
}

/*
* about Mecanum wheel
* leftFront Wheel = Y + R + X
* leftRear Wheel = Y + R - X
* rightFront Wheel = Y - R - X
* rightRear Wheel = Y - R + X
*
* X = (LF - LR) / 2
* Y = LF + RF = LR + RR
* R = (LF - RR) / 2
* */