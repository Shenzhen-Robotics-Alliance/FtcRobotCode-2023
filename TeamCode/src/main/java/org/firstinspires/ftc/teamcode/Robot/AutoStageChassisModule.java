package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoStageChassisModule {
    private final double encoderCorrectionFactor = -1;
    private final boolean x_y_Reversed = true;

    private double[] dynamicalEncoderCorrectionBias = new double[4]; // the leftFront, leftRear, rightFront and rightRear encoder correction


    private final double rotationDeviationTolerance = Math.toRadians(5);
    private final double rotationDifferenceStartDecelerating = Math.toRadians(45);
    private final double minRotatingPower = 0.15;
    private final double stableRotatingPower = 0.35;

    private final double positionDeviationTolerance = 128;
    private final double distanceStartDecelerating = 512; // TODO set these two values to be some small encoder values
    private final double minMotioningPower = 0.1;
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

        System.out.print("power:");
        System.out.println(ChassisModule.linearMap(
                positionDeviationTolerance,
                distanceStartDecelerating,
                minMotioningPower,
                stableMotioningPower,
                2000));
    }

    public void initRobotChassis() {
        imu.calibrateIMUHeading();
        this.imuReaderThread = new Thread(() -> {
            while (!isStopRequested) imu.updateIMUStatus();
        });
        imuReaderThread.start();
        this.calibrateEncoder();
    }

    public void moveRobotWithEncoder(double targetedXPosition, double targetedYPosition) {
        // reposition the distance of a given value, in reference to the robot itself before motion
        calculateStartingEncoderPosition();
        setRobotPosition(targetedXPosition, targetedYPosition);
    }

    public void setRobotPosition(double targetedXPosition, double targetedYPosition) {
        // set the running parameters for each motors
        this.driver.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.driver.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.driver.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.driver.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double distanceXPosition, distanceYPosition, distanceLeft;
        do {
            // get the distance left in x and y position
            double[] robotCurrentPosition = getEncoderPosition();
            distanceXPosition = targetedXPosition - robotCurrentPosition[0];
            distanceYPosition = targetedYPosition - robotCurrentPosition[1];

            // calculate, according to the distance left, using linear mapping, the needed motor power
            double xVelocity = ChassisModule.linearMap(
                    positionDeviationTolerance,
                    distanceStartDecelerating,
                    minMotioningPower,
                    stableMotioningPower,
                    distanceXPosition);
            double yVelocity = ChassisModule.linearMap(
                    positionDeviationTolerance,
                    distanceStartDecelerating,
                    minMotioningPower,
                    stableMotioningPower,
                    distanceYPosition
            );

            driver.leftFront.setPower(yVelocity + xVelocity);
            driver.leftRear.setPower(yVelocity - xVelocity);
            driver.rightFront.setPower(yVelocity - xVelocity);
            driver.rightRear.setPower(yVelocity + xVelocity);

            distanceLeft = Math.sqrt(distanceXPosition*distanceXPosition + distanceYPosition*distanceYPosition);

            System.out.print(distanceXPosition); System.out.print(" "); System.out.println(xVelocity);
        } while(distanceLeft > positionDeviationTolerance);
    }

    public void setRobotXPosition(double targetedXPosition) {
        // set the running parameters for each motors
        this.driver.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.driver.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.driver.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.driver.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double distanceXPosition, distanceLeft;
        do {
            // get the distance left in x and y position
            double[] robotCurrentPosition = getEncoderPosition();
            distanceXPosition = targetedXPosition - robotCurrentPosition[0];

            // calculate, according to the distance left, using linear mapping, the needed motor power
            double xVelocity = ChassisModule.linearMap(
                    positionDeviationTolerance,
                    distanceStartDecelerating,
                    minMotioningPower,
                    stableMotioningPower,
                    distanceXPosition);

            driver.leftFront.setPower(xVelocity);
            driver.leftRear.setPower(-xVelocity);
            driver.rightFront.setPower(-xVelocity);
            driver.rightRear.setPower(xVelocity);

            distanceLeft = Math.abs(distanceXPosition);

            System.out.print(distanceXPosition); System.out.print(" "); System.out.println(xVelocity);
        } while(distanceLeft > positionDeviationTolerance);
    }

    public void setRobotYPosition(double targetedYPosition) {
        // set the running parameters for each motors
        this.driver.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.driver.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.driver.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.driver.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double distanceYPosition, distanceLeft;
        do {
            // get the distance left in x and y position
            double[] robotCurrentPosition = getEncoderPosition();
            distanceYPosition = targetedYPosition - robotCurrentPosition[0];

            // calculate, according to the distance left, using linear mapping, the needed motor power
            double yVelocity = ChassisModule.linearMap(
                    positionDeviationTolerance,
                    distanceStartDecelerating,
                    minMotioningPower,
                    stableMotioningPower,
                    distanceYPosition);

            driver.leftFront.setPower(yVelocity);
            driver.leftRear.setPower(yVelocity);
            driver.rightFront.setPower(yVelocity);
            driver.rightRear.setPower(yVelocity);

            distanceLeft = Math.abs(distanceYPosition);

            System.out.print(distanceYPosition); System.out.print(" "); System.out.println(yVelocity);
        } while(distanceLeft > positionDeviationTolerance);
    }

    public void calibrateEncoder() { calculateStartingEncoderPosition(); }

    private void calculateStartingEncoderPosition() {
        // calculate the current position with encoder data, using the algorithm of Mecanum wheel
        encoderStartingPosition = getEncoderPosition();
        encoderStartingRotation = getEncoderRotation();
    }

    public double[] getEncoderPosition() {
        // get the encoder value and calculate the position using Mecanum wheel algorithm
        double[] encoderPosition = new double[2];
        if (x_y_Reversed) { // if the x and y axles are reversed
            this.encoderCurrentPosition[1] = (
                    (double)
                            this.driver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            + this.driver.rightFront.getCurrentPosition() * encoderCorrectionFactor)
                    / 2;
            this.encoderCurrentPosition[0] = (
                    (double) this.driver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            - this.driver.leftRear.getCurrentPosition() * encoderCorrectionFactor)
                    / 2;        } else {
            this.encoderCurrentPosition[0] = (
                    (double)
                            this.driver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            + this.driver.rightFront.getCurrentPosition() * encoderCorrectionFactor)
                / 2;
            this.encoderCurrentPosition[1] = (
                    (double) this.driver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            - this.driver.leftRear.getCurrentPosition() * encoderCorrectionFactor)
                / 2;
        }

        // calculate the position
        encoderPosition[0] = encoderCurrentPosition[0] - encoderStartingPosition[0];
        encoderPosition[1] = encoderCurrentPosition[1] - encoderStartingPosition[1];
        return encoderPosition;
    }

    public double getEncoderRotation() {
        double encoderRotation;
        this.encoderCurrentRotation = (double) this.driver.leftFront.getCurrentPosition()*encoderCorrectionFactor - this.driver.rightRear.getCurrentPosition()*encoderCorrectionFactor;

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
                    rotationDeviationTolerance, rotationDifferenceStartDecelerating, minRotatingPower, stableRotatingPower, clockWiseDifference
            ); // set the speed of rotation depending on the distance left, start to slow down when the difference is smaller than 90deg
            setRobotMotion(0, 0, rotatingSpeed);
            System.out.print("clockwise difference: ");
            System.out.println(clockWiseDifference);
        } while (clockWiseDifference > Math.toRadians(5) & !isStopRequested);
        setRobotMotion(0, 0, 0);
    }

    private void rotateCounterClockWise(double targetedRotation) {
        double counterClockWiseDifference ;
        do {
            double currentRotation = imu.getRobotHeading();
            if (targetedRotation < currentRotation) counterClockWiseDifference = targetedRotation - currentRotation;
            else counterClockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of counter-clockwise difference

            double rotatingSpeed = ChassisModule.linearMap(
                    rotationDeviationTolerance, rotationDifferenceStartDecelerating, minRotatingPower, stableRotatingPower, counterClockWiseDifference
                    ) *-1;
            setRobotMotion(0, 0, rotatingSpeed);
            System.out.print("counter-clockwise difference: ");
            System.out.println(counterClockWiseDifference);
        } while (counterClockWiseDifference > Math.toRadians(5) & !isStopRequested);
        setRobotMotion(0, 0, 0);
    }

    private void setRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        // control the Mecanum wheel
        driver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        driver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
    }

    private void correctEncoderValueUsingIMU() {
        // TODO write this method to correct the encoder with IMu, which is far more accurate
        // calculate the difference between the actual encoder value and the expected ones using the IMU, which is more accurate
        imu.updateIMUStatus();
    }

    public double getImuYaw() {
        return imu.getRobotHeading();
    }

    public void terminate() {
        isStopRequested = true;
        setRobotMotion(0, 0, 0);
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
* Y = (LF + RF) / 2 = (LR + RR) / 2
* R = (LF - RR) / 2
* */