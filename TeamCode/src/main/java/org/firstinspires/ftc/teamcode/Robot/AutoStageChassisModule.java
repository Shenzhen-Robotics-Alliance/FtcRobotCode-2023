package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoStageChassisModule {
    private final double acceptedRotationDeviation = Math.toRadians(5);
    private final double rotationDifferenceStartDecelerating = Math.toRadians(45);
    private final double minRotatingPower = 0.15;
    private final double stableRotatingPower = 0.35;

    private final double acceptedPositionDeviation = 10;
    private final double distanceStartDecelerating = 100; // TODO set these two values to be some small encoder values
    private final double minMotioningPower = 0.15;
    private final double stableMotioningPower = 0.35;

    private HardwareDriver driver;
    private final IMUReader imu;
    private Thread imuReaderThread;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;


    private double encoderStartingRotation;
    private double[] encoderStartingPosition = new double[2];

    private double[] encoderCurrentPosition = new double[2];
    private double encoderCurrentRotation;
    private boolean isStopRequested = false;


    public AutoStageChassisModule(HardwareDriver driver, HardwareMap hardwareMap) {
        this.driver = driver;
        this.imu = new IMUReader(hardwareMap); // use backup imu2 from extension hub if imu does not work
        this.fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
    }
    public AutoStageChassisModule(HardwareDriver driver, HardwareMap hardwareMap, ComputerVisionFieldNavigation_v2 fieldNavigation) {
        this.driver = driver;
        this.imu = new IMUReader(hardwareMap); // use backup imu2 from extension hub if imu does not work
        this.fieldNavigation = fieldNavigation;
    }

    public void initRobotChassis() {
        imu.calibrateIMUHeading();
        this.imuReaderThread = new Thread(() -> {
            while (!isStopRequested) imu.updateIMUStatus();
        });
        imuReaderThread.start();
    }

    public void moveRobotWithEncoder(double targetedXPosition, double targetedYPosition) {
        // reposition the distance of a given value, in reference to the robot itself before motion
        calculateStartingEncoderPosition();
        setRobotPosition(targetedXPosition, targetedYPosition);
    }

    public void setRobotPosition(double targetedXPosition, double targetedYPosition) {
        // move to the requested targeted position, in reference to the starting position

        // calculate the required movement to get to the objective position
        double[] requiredMovement = new double[2];
        requiredMovement[0] = targetedXPosition - encoderStartingPosition[0]; requiredMovement[1] = targetedYPosition - encoderStartingPosition[1];

        // set the targeted position for each motor
        this.driver.leftFront.setTargetPosition((int) (requiredMovement[1] + encoderStartingRotation + requiredMovement[0]));
        this.driver.leftRear.setTargetPosition((int) (requiredMovement[1] + encoderStartingRotation - requiredMovement[0]));
        this.driver.rightFront.setTargetPosition((int) (requiredMovement[1] - encoderStartingRotation - requiredMovement[0]));
        this.driver.rightRear.setTargetPosition((int) (requiredMovement[1] - encoderStartingRotation + requiredMovement[0]));

        // set the running parameters for each motors
        this.driver.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.driver.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.driver.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.driver.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.driver.leftFront.setPower(stableMotioningPower);
        this.driver.leftRear.setPower(stableMotioningPower);
        this.driver.rightFront.setPower(stableMotioningPower);
        this.driver.rightRear.setPower(stableMotioningPower);

        double xPositionDifferent; double yPositionDifferent;
        double distanceLeft;
        double power;
        do {
            getEncoderPosition();
            xPositionDifferent = targetedXPosition - encoderCurrentPosition[0];
            yPositionDifferent = targetedYPosition - encoderCurrentPosition[1];
            distanceLeft = Math.sqrt(xPositionDifferent*xPositionDifferent + yPositionDifferent*yPositionDifferent);
            power = ChassisModule.linearMap(acceptedPositionDeviation, distanceStartDecelerating, minMotioningPower, stableMotioningPower, distanceLeft);
            this.driver.leftFront.setPower(power);
            this.driver.leftRear.setPower(power);
            this.driver.rightFront.setPower(power);
            this.driver.rightRear.setPower(power);

        } while (Math.abs(distanceLeft) > acceptedPositionDeviation); // wait until the process is done, accept a small amount of error
    }

    public void calibrateEncoder() { calculateStartingEncoderPosition(); }

    private void calculateStartingEncoderPosition() {
        // calculate the current position with encoder data, using the algorithm of Mecanum wheel
        this.encoderStartingPosition[0] = this.driver.leftFront.getCurrentPosition() + this.driver.rightFront.getCurrentPosition();
        this.encoderStartingPosition[1] = this.driver.leftFront.getCurrentPosition() - this.driver.leftRear.getCurrentPosition();
        this.encoderStartingRotation = this.driver.leftFront.getCurrentPosition() - this.driver.rightRear.getCurrentPosition();
    }

    public double[] getEncoderPosition() {
        double[] encoderPosition = new double[2];
        this.encoderCurrentPosition[0] = (double) this.driver.leftFront.getCurrentPosition() + this.driver.rightFront.getCurrentPosition();
        this.encoderCurrentPosition[1] = (double) this.driver.leftFront.getCurrentPosition() - this.driver.leftRear.getCurrentPosition();

        encoderPosition[0] = encoderCurrentPosition[0] - encoderStartingPosition[0];
        encoderPosition[1] = encoderCurrentPosition[1] - encoderStartingPosition[1];
        return encoderPosition;
    }

    public double getEncoderRotation() {
        double encoderRotation;
        this.encoderCurrentRotation = (double) this.driver.leftFront.getCurrentPosition() - this.driver.rightRear.getCurrentPosition();

        encoderRotation = encoderCurrentRotation - encoderStartingRotation;
        return encoderRotation;
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