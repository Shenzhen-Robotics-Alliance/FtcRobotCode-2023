/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: AutoStageChassisModule.java
 *
 * the program that controls the moving of the robot in the auto stage
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */
package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

public class AutoStageRobotChassis extends RobotModule {
    // presets for rotation correcting
    private final double encoderCorrectionFactor = -1;
    private final boolean x_y_Reversed = true;
    private final boolean useIMUCorrection = false;
    private final boolean runWithEncoder = false;
    private final boolean rotationCorrecting = false;

    private double[] dynamicalEncoderCorrectionBias = new double[4]; // the leftFront, leftRear, rightFront and rightRear encoder correction

    // presets for position correction
    private final double rotationDeviationTolerance = Math.toRadians(5);
    private final double rotationDifferenceStartDecelerating = Math.toRadians(45);
    private final double minRotatingPower = 0.25;
    private final double stableRotatingPower = 0.45;
    private final double minRotationEncoderVelocity = 80;
    private final double stableRotatingEncoderVelocity = 200;
    private final double encoderRotationPerRadian = 3900 / (Math.PI*2);

    private final double positionDeviationTolerance = 60;
    private final double distanceStartDecelerating = 350;
    private final double minMotioningPower = 0.25;
    private final double stableMotioningPower = 0.4;
    private final double minMotioningEncoderVelocity = 80;
    private final double stableMotioningEncoderVelocity = 200;

    private final double minDifferenceToToleranceRatio = 1.6;
    private final double minRotatingCorrectionPower = 0.05;

    // constant for visual navigation
    private final double encoderValuePerVisualNavigationValue = 650 / 400; // during the test, visual module coordinate increase by 400, encoder increase by -650
    private final double waitForNavigationSignTimeLimitation = 0.5; // the time limit when waiting for navigation sign to show up
    private final double rotationalMotionCorrectionFactor = -1;

    private HardwareDriver hardwareDriver;
    private final IMUReader imuReader;
    private Thread imuReaderThread;
    private ComputerVisionFieldNavigation_v2 fieldNavigation;
    private Thread fieldNavigationThread;


    private double encoderStartingRotation;
    private double[] encoderStartingPosition = new double[2];

    private double[] encoderCurrentPosition = new double[2];
    private double encoderCurrentRotation;
    private boolean isStopRequested = false;


    public AutoStageRobotChassis(HardwareDriver hardwareDriver, HardwareMap hardwareMap) {
        this.hardwareDriver = hardwareDriver;
        this.imuReader = new IMUReader(hardwareMap); // use backup imu2 from extension hub if imu does not work
        this.fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap);
        this.fieldNavigationThread = new Thread(fieldNavigation);
    }
    public AutoStageRobotChassis(HardwareDriver hardwareDriver, HardwareMap hardwareMap, ComputerVisionFieldNavigation_v2 fieldNavigation) {
        this.hardwareDriver = hardwareDriver;
        this.imuReader = new IMUReader(hardwareMap); // use backup imu2 from extension hub if imu does not work
        if (fieldNavigation == null) fieldNavigation = new ComputerVisionFieldNavigation_v2(hardwareMap); System.out.println("init field navigation");
        this.fieldNavigation = fieldNavigation;
        this.fieldNavigationThread = new Thread(fieldNavigation);
    }

    public AutoStageRobotChassis(HardwareDriver hardwareDriver, HardwareMap hardwareMap, ComputerVisionFieldNavigation_v2 fieldNavigation, IMUReader imuReader) {
        this.hardwareDriver = hardwareDriver;
        this.imuReader = imuReader;
        this.fieldNavigation = fieldNavigation;
        this.fieldNavigationThread = new Thread(fieldNavigation);
    }

    /**
     * construct method for chassis module used in autonomous stage
     */
    public AutoStageRobotChassis() {
        /* call to super method */
        super("autoStageRobotChassis");
    }

    /**
     * initialize the chassis module for autonomous stage
     * @param dependentModules this module have the following modules as optional(pass them in the form of hashmap, or they will be created in the initialization):
     *                         "fieldNavigation" : ComputerVisionFieldNavigation_v2, the module that calculates the robot's position using cameras;
     *                         "imuReader" : IMUReader, the module that reads and formats the data from built-in imu sensor in the robot;
     *@param dependentInstances this module needs the following instances(pass them in the form of hashmap):
     *                          "hardwareDriver" : HardwareDriver, the driver of the robot's hardware, gain from main program;
     *                          "hardwareMap" : HardwareMap, the connection to the ports of the robot;
     */
    @Override
    public void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) throws NullPointerException {
        /* throw out an exception if the map of all the necessary instances are given as empty */
        if (dependentInstances.isEmpty()) throw new NullPointerException (
                "an empty set of dependent instance given to the module<<" + this.getModuleName() + ">> which requires at least one instance(s)"
        );

        /* throw out an exception if a necessary instance */
        if (!dependentInstances.containsKey("hardwareDriver")) throw new NullPointerException(
                "required dependency <<" + "hardwareDriver" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentInstances.containsKey("hardwareMap")) throw new NullPointerException(
                "required dependency <<" + "hardwareMap" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the instances from the param */
        this.hardwareDriver = (HardwareDriver) dependentInstances.get("hardwareDriver");
        HardwareMap hardwareMap = (HardwareMap) dependentInstances.get("hardwareMap");

        /* get the field navigation module, if specified, from the param,
        *  otherwise, create the modules
        * */
        if (dependentModules.containsKey("fieldNavigation")) this.fieldNavigation = (ComputerVisionFieldNavigation_v2) dependentModules.get("fieldNavigation");
        else {
            /* pass the hardware ports to the field navigation module */
            HashMap<String, RobotModule> fieldNavigationDependentModules = null;
            HashMap<String, Object> fieldNavigationDependentInstances = new HashMap<>(1);
            fieldNavigationDependentInstances.put("hardwareMap", hardwareMap);
            fieldNavigation = new ComputerVisionFieldNavigation_v2();
            fieldNavigation.init(fieldNavigationDependentModules, fieldNavigationDependentInstances);
        }
        /* same for the imu module */
        if (dependentInstances.containsKey("imuReader")) this.imuReader = (IMUReader) dependentModules.get("imuReader");

        /* calibrate the encoders and the imu */
        imuReader.calibrateIMU();
        this.calibrateEncoder();
    }

    /**
     * update an instance used in the module
     *
     * @Deprecated the robot chassis in auto stage does not support changing modules after initialization
     */
    @Override @Deprecated public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    @Override
    public void periodic() throws InterruptedException {
        imuReader.updateIMUStatus();
    }

    public void initRobotChassis() {
        imuReader.calibrateIMU();
        this.imuReaderThread = new Thread(() -> {
            while (!isStopRequested) imuReader.updateIMUStatus();
        });
        imuReaderThread.start();
        this.calibrateEncoder();
        fieldNavigationThread.start();
    }

    public void moveRobotWithEncoder(double targetedXPosition, double targetedYPosition) {
        // reposition the distance of a given value, in reference to the robot itself before motion
        calculateStartingEncoderPosition();
        setRobotPosition(targetedXPosition, targetedYPosition);
    }

    public void setRobotPosition(double targetedXPosition, double targetedYPosition) {
        // check for termination
        if (isStopRequested) return;

        // get the rotation at the start of the motion
        double startingRotation;
        if (useIMUCorrection) startingRotation = getImuYaw();
        else startingRotation = getEncoderRotation();

        double distanceXPosition, distanceYPosition;
        double rotationDeviationDuringProcess, dynamicalRotationCorrection;
        boolean deviationAccepted;
        do {
            // get the distance left in x and y position
            double[] robotCurrentPosition = getEncoderPosition();
            distanceXPosition = targetedXPosition - robotCurrentPosition[0];
            distanceYPosition = targetedYPosition - robotCurrentPosition[1];

            // calculate, according to the distance left, using linear mapping, the needed motor power
            double xVelocity, yVelocity;
            if (runWithEncoder) {
                xVelocity = getMotioningEncoderVelocity(distanceXPosition);
                yVelocity = getMotioningEncoderVelocity(distanceYPosition);
            } else {
                xVelocity = getMotioningPower(distanceXPosition);
                yVelocity = getMotioningPower(distanceYPosition);
            }

            // correct the rotation using IMU or encoder, depending on the presets
            if (useIMUCorrection) rotationDeviationDuringProcess = startingRotation - getImuYaw();
            else rotationDeviationDuringProcess = startingRotation - getEncoderRotation();
            if (runWithEncoder) dynamicalRotationCorrection = getRotatingEncoderVelocity(rotationDeviationDuringProcess);
            else dynamicalRotationCorrection = getRotatingPower(rotationDeviationDuringProcess);

            // set the motor power for the robot
            if(rotationCorrecting) setRobotMotion(xVelocity, yVelocity, dynamicalRotationCorrection);
            else setRobotMotion(xVelocity, yVelocity, 0);

            // whether the deviation is acceptable
            deviationAccepted = Math.abs(distanceXPosition) < positionDeviationTolerance*minDifferenceToToleranceRatio
                             && Math.abs(distanceYPosition) < positionDeviationTolerance*minDifferenceToToleranceRatio;

            System.out.print(getImuYaw()); System.out.print(" "); System.out.println(imuReader.getRobotHeading());
        } while(!deviationAccepted && !isStopRequested);

        setRobotMotion(0, 0, 0);
    }

    public void setRobotPositionUsingIMU(double targetedXPosition, double targetedYPosition) {
        // check for termination
        if (isStopRequested) return;

        // get the rotation at the start of the motion
        double startingRotation;
        if (useIMUCorrection) startingRotation = getImuYaw();
        else startingRotation = getEncoderRotation();

        double distanceXPosition, distanceYPosition;
        double rotationDeviationDuringProcess, dynamicalRotationCorrection;
        boolean deviationAccepted;
        do {
            // get the distance left in x and y position
            double[] robotCurrentPosition = imuReader.getIMUPosition();
            distanceXPosition = targetedXPosition - robotCurrentPosition[0];
            distanceYPosition = targetedYPosition - robotCurrentPosition[1];

            // calculate, according to the distance left, using linear mapping, the needed motor power
            double xVelocity, yVelocity;
            if (runWithEncoder) {
                xVelocity = getMotioningEncoderVelocity(distanceXPosition);
                yVelocity = getMotioningEncoderVelocity(distanceYPosition);
            } else {
                xVelocity = getMotioningPower(distanceXPosition);
                yVelocity = getMotioningPower(distanceYPosition);
            }

            // correct the rotation using IMU or encoder, depending on the presets
            if (useIMUCorrection) rotationDeviationDuringProcess = startingRotation - getImuYaw();
            else rotationDeviationDuringProcess = startingRotation - getEncoderRotation();
            if (runWithEncoder) dynamicalRotationCorrection = getRotatingEncoderVelocity(rotationDeviationDuringProcess);
            else dynamicalRotationCorrection = getRotatingPower(rotationDeviationDuringProcess);

            // set the motor power for the robot
            if(rotationCorrecting) setRobotMotion(xVelocity, yVelocity, dynamicalRotationCorrection);
            else setRobotMotion(xVelocity, yVelocity, 0);

            // whether the deviation is acceptable
            deviationAccepted = Math.abs(distanceXPosition) < positionDeviationTolerance*minDifferenceToToleranceRatio
                    && Math.abs(distanceYPosition) < positionDeviationTolerance*minDifferenceToToleranceRatio;

            System.out.print(getEncoderPosition()); System.out.print(" "); System.out.println(imuReader.getIMUPosition());
        } while(!deviationAccepted && !isStopRequested);

        setRobotMotion(0, 0, 0);
    }

    private double getMotioningEncoderVelocity(double encoderDifference) {
        if (Math.abs(encoderDifference) < positionDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                positionDeviationTolerance * minDifferenceToToleranceRatio,
                distanceStartDecelerating,
                minMotioningEncoderVelocity,
                stableMotioningEncoderVelocity,
                encoderDifference);
    }

    private double getMotioningPower(double encoderDifference) {
        if (Math.abs(encoderDifference) < positionDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                positionDeviationTolerance * minDifferenceToToleranceRatio,
                distanceStartDecelerating,
                minMotioningPower,
                stableMotioningPower,
                encoderDifference);
    }

    private double getRotatingEncoderVelocity(double rotationDifference) {
        if (Math.abs(rotationDifference) < rotationDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                rotationDeviationTolerance * minDifferenceToToleranceRatio,
                rotationDifferenceStartDecelerating,
                minRotationEncoderVelocity,
                stableRotatingEncoderVelocity,
                rotationDifference);
    }

    private double getRotatingPower(double rotationDifference) {
        if (Math.abs(rotationDifference) < rotationDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                rotationDeviationTolerance * minDifferenceToToleranceRatio,
                rotationDifferenceStartDecelerating,
                minRotatingPower,
                stableRotatingPower,
                rotationDifference);
    }

    private double getRotatingCorrectionPower(double rotationDifference) {
        if (Math.abs(rotationDifference) < rotationDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                rotationDeviationTolerance * minDifferenceToToleranceRatio,
                rotationDifferenceStartDecelerating,
                minRotatingCorrectionPower,
                stableRotatingPower,
                rotationDifference);
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
                            this.hardwareDriver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            + this.hardwareDriver.rightFront.getCurrentPosition() * encoderCorrectionFactor)
                    / 2;
            this.encoderCurrentPosition[0] = (
                    (double) this.hardwareDriver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            - this.hardwareDriver.leftRear.getCurrentPosition() * encoderCorrectionFactor)
                    / 2;        } else {
            this.encoderCurrentPosition[0] = (
                    (double)
                            this.hardwareDriver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            + this.hardwareDriver.rightFront.getCurrentPosition() * encoderCorrectionFactor)
                / 2;
            this.encoderCurrentPosition[1] = (
                    (double) this.hardwareDriver.leftFront.getCurrentPosition() * encoderCorrectionFactor
                            - this.hardwareDriver.leftRear.getCurrentPosition() * encoderCorrectionFactor)
                / 2;
        }

        // calculate the position
        encoderPosition[0] = encoderCurrentPosition[0] - encoderStartingPosition[0];
        encoderPosition[1] = encoderCurrentPosition[1] - encoderStartingPosition[1];
        return encoderPosition;
    }

    public double getEncoderRotation() {
        double encoderRotation, robotRotation;
        this.encoderCurrentRotation = (double) this.hardwareDriver.leftFront.getCurrentPosition()*encoderCorrectionFactor - this.hardwareDriver.rightRear.getCurrentPosition()*encoderCorrectionFactor;

        encoderRotation = encoderCurrentRotation - encoderStartingRotation;

        robotRotation = encoderRotation / encoderRotationPerRadian;
        robotRotation %= Math.PI * 2;
        return robotRotation;
    }

    public void setRobotRotation(double targetedRotation) { // rote the robot to targeted spot, in radian
        // check for termination
        if (isStopRequested) return;

        final double fullCircle = 2 * Math.PI;
        double currentRotation = imuReader.getRobotHeading();
        if (currentRotation < 0) currentRotation = fullCircle + currentRotation;

        double numericalRotationDifference = targetedRotation - currentRotation;
        double counterClockWiseDifference, clockWiseDifference;

        System.out.print("robot facing:");
        System.out.println(currentRotation);
        if (numericalRotationDifference > 0) { // when the target is at the positive(clockwise) direction
            clockWiseDifference = -numericalRotationDifference;
            double targetedToOrigin = fullCircle - targetedRotation;
            counterClockWiseDifference = targetedToOrigin + currentRotation;
        } else {
            counterClockWiseDifference = numericalRotationDifference;
            double targetedToOrigin = fullCircle - targetedRotation;
            clockWiseDifference = targetedToOrigin + currentRotation;
        }

        if (clockWiseDifference < counterClockWiseDifference) {
            rotateClockWise(clockWiseDifference);
        } else {
            rotateCounterClockWise(counterClockWiseDifference);
        }
    }

    private void rotateClockWise(double targetedRotation) {
        // check for termination
        if (isStopRequested) return;

        double clockWiseDifference ;
        do {
            double currentRotation = imuReader.getRobotHeading();
            if (targetedRotation > currentRotation) clockWiseDifference = targetedRotation - currentRotation;
            else clockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of clockwise difference

            double rotatingSpeed = getRotatingPower(clockWiseDifference); // set the speed of rotation depending on the distance left, start to slow down when the difference is smaller than 90deg
            setRobotMotion(0, 0, rotatingSpeed);
        } while (clockWiseDifference > Math.toRadians(5) & !isStopRequested);
        setRobotMotion(0, 0, 0);
    }

    private void rotateCounterClockWise(double targetedRotation) {
        // check for termination
        if (isStopRequested) return;

        double counterClockWiseDifference ;
        do {
            double currentRotation = imuReader.getRobotHeading();
            if (targetedRotation < currentRotation) counterClockWiseDifference = targetedRotation - currentRotation;
            else counterClockWiseDifference = 2*Math.PI - targetedRotation + currentRotation; // repeat the calculation of counter-clockwise difference

            double rotatingSpeed = getRotatingPower(counterClockWiseDifference) *-1;
            setRobotMotion(0, 0, rotatingSpeed);
        } while (counterClockWiseDifference > Math.toRadians(5) & !isStopRequested);
        setRobotMotion(0, 0, 0);
    }

    public void setRobotPositionWithVisualNavigation(double targetedXPosition, double targetedYPosition) {
        // check for termination
        if (isStopRequested) return;

        // wait for the navigation sign to show up
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        // while (elapsedTime.seconds() < waitForNavigationSignTimeLimitation) {
        while (elapsedTime.seconds() < 10) {
            Thread.yield();
            if (fieldNavigation.checkNavigationSignsVisibility()) break;
        }
        System.out.println("navigation sign detected");

        // move to the targeted position, using visual guidance
        // get the rotation at the start of the motion
        double startingRotation;
        if (useIMUCorrection) startingRotation = getImuYaw();
        else startingRotation = getEncoderRotation();

        double distanceXPosition, distanceYPosition, distanceLeft;
        double rotationDeviationDuringProcess, dynamicalRotationCorrection;
        do {
            // get the distance left in x and y position
            double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
            distanceXPosition = targetedXPosition - robotCurrentPosition[0];
            distanceYPosition = targetedYPosition - robotCurrentPosition[1];

            // calculate, according to the distance left, using linear mapping and visual navigation, the needed motor power
            double xVelocity, yVelocity;
            if (runWithEncoder) {
                xVelocity = getVisualGuidanceMotioningEncoderVelocity(distanceXPosition);
                yVelocity = getVisualGuidanceMotioningEncoderVelocity(distanceYPosition);
            } else {
                xVelocity = getVisualGuidanceMotioningPower(distanceXPosition);
                yVelocity = getVisualGuidanceMotioningPower(distanceYPosition);
            }

            // correct the rotation using IMU or encoder, depending on the presets
            if (useIMUCorrection) rotationDeviationDuringProcess = startingRotation - getImuYaw();
            else rotationDeviationDuringProcess = startingRotation - getEncoderRotation();
            if (runWithEncoder) dynamicalRotationCorrection = getRotatingEncoderVelocity(rotationDeviationDuringProcess);
            else dynamicalRotationCorrection = getRotatingPower(rotationDeviationDuringProcess);

            // set the motor power for the robot
            if(rotationCorrecting) setRobotMotion(xVelocity, yVelocity, dynamicalRotationCorrection);
            else setRobotMotion(xVelocity, yVelocity, 0);

            // calculate the distance left, for further judgement on whether to stick with the loop or not
            distanceLeft = Math.sqrt(distanceXPosition*distanceXPosition + distanceYPosition*distanceYPosition);
            System.out.print(xVelocity); System.out.print(" "); System.out.print(yVelocity); System.out.print(" "); System.out.println(rotationCorrecting);
        } while(distanceLeft > positionDeviationTolerance && fieldNavigation.PositionLastUpdate() < 0.5 && !isStopRequested); // cut the loop if the process is finished or if the navigation signs are out of sight

        setRobotMotion(0, 0, 0);
    }

    private double getVisualGuidanceMotioningEncoderVelocity(double visualNavigationDifference) {
        if (Math.abs(visualNavigationDifference) < positionDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                positionDeviationTolerance * minDifferenceToToleranceRatio,
                distanceStartDecelerating,
                minMotioningEncoderVelocity,
                stableMotioningEncoderVelocity,
                visualNavigationDifference * encoderValuePerVisualNavigationValue
        );
    }

    private double getVisualGuidanceMotioningPower(double visualNavigationDifference) {
        if (Math.abs(visualNavigationDifference) < positionDeviationTolerance) return 0; // debug the auto correction
        return RobotChassis.linearMap(
                positionDeviationTolerance * minDifferenceToToleranceRatio,
                distanceStartDecelerating,
                minMotioningPower,
                stableMotioningPower,
                visualNavigationDifference * encoderValuePerVisualNavigationValue
        );
    }

    private void setRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        rotationalMotion *= rotationalMotionCorrectionFactor;
        // control the Mecanum wheel
        if (runWithEncoder) {
            hardwareDriver.leftFront.setVelocity(yAxleMotion + rotationalMotion + xAxleMotion);
            hardwareDriver.leftRear.setVelocity(yAxleMotion + rotationalMotion - xAxleMotion);
            hardwareDriver.rightFront.setVelocity(yAxleMotion - rotationalMotion - xAxleMotion);
            hardwareDriver.rightRear.setVelocity(yAxleMotion - rotationalMotion + xAxleMotion);

            this.hardwareDriver.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardwareDriver.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardwareDriver.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardwareDriver.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            // set the running parameters for each motors
            this.hardwareDriver.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.hardwareDriver.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.hardwareDriver.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.hardwareDriver.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hardwareDriver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
            hardwareDriver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
            hardwareDriver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
            hardwareDriver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
        }
    }

    private void correctEncoderValueUsingIMU() {
        // check for termination
        if (isStopRequested) return;

        // TODO write this method to correct the encoder with IMu, which is far more accurate
        // calculate the difference between the actual encoder value and the expected ones using the IMU, which is more accurate
        imuReader.updateIMUStatus();
    }

    public double getImuYaw() {
        return imuReader.getRobotHeading();
    }

    public void testRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        setRobotMotion(xAxleMotion, yAxleMotion, rotationalMotion);
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