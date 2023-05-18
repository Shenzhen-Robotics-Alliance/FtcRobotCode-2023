package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModules.RobotPositionCalculator;

public class ChassisDriver {
    /* private final double maxPower = 0.6;
    private final double encoderDistanceStartDecelerate = 15000;
    private final double motorPowerPerEncoderValueError = (maxPower / encoderDistanceStartDecelerate); */
    private final double maxRotatingPower = 0.6;
    private final double rotationDifferenceStartDecelerate = Math.toRadians(15);
    private final double motorPowerPerRotationDifference = -(maxRotatingPower / rotationDifferenceStartDecelerate);
    private final double velocityDebugTimeRotation = 0.05;
    private final double integralCoefficientRotation = 0; // not needed yet

    private final double maxMotioningPower = 0.5;
    private final double encoderDifferenceStartDecelerate = 10;
    private final double motorPowerPerEncoderDifference = (maxMotioningPower / encoderDifferenceStartDecelerate);
    private final double velocityDebugTimeTranslation = 0.1;
    private final double integrationCoefficientTranslation = 0; // not needed yet

    private HardwareDriver hardwareDriver;
    private RobotPositionCalculator positionCalculator;

    private double xAxleMotion = 0;
    private double yAxleMotion = 0;
    private double rotationalMotion = 0;

    private double xAxleTranslationTarget = 0;

    private double yAxleTranslationTarget = 0;
    private double[] translationalIntegration = new double[2];
    private double targetedRotation = 0;
    private boolean RASActivation = false;

    private final int goToRotationMode = 1;
    private final int manualMode = 0;
    private int rotationMode = manualMode;
    private final int gotoPositionMode = 2;
    private int translationalMode = manualMode;

    private double rotationalIntegration;

    private ElapsedTime dt = new ElapsedTime();

    public ChassisDriver(HardwareDriver hardwareDriver, RobotPositionCalculator positionCalculator) {
        this.hardwareDriver = hardwareDriver;
        this.positionCalculator = positionCalculator;
    }

    public void setRobotTranslationalMotion(double xAxleMotion, double yAxleMotion) {
        this.xAxleMotion = xAxleMotion;
        this.yAxleMotion = yAxleMotion;
        sendCommandsToMotors();
    }

    public void setTargetedTranslation(double xAxleTranslation, double yAxleTranslation) {
        this.xAxleTranslationTarget = xAxleTranslation;
        this.yAxleTranslationTarget = yAxleTranslation;
        switchToGoToPositionMode();;
        translationalIntegration[0] = 0;
        translationalIntegration[1] = 0;
    }

    /** in radian */
    public void setTargetedRotation(double targetedRotation) {
        this.targetedRotation = targetedRotation;
        switchToGoToRotationMode();
        rotationalIntegration = 0;
    }

    public void setRotationalMotion(double rotationalMotion) {
        switchToManualRotationMode();
        this.rotationalMotion = rotationalMotion;
        dt.reset();
        sendCommandsToMotors();
    }

    public void pilotInterruption() {
        RASActivation = false;
    }

    public void newAimStarted() {
        RASActivation = true;
    }

    public void aimStopped() {RASActivation = false; }

    public void switchToManualRotationMode() { rotationMode = manualMode;}
    private void switchToGoToRotationMode() { rotationMode = goToRotationMode; }

    public void switchToManualPositionMode() { translationalMode = manualMode; }
    private void switchToGoToPositionMode() {
        translationalMode = gotoPositionMode;
        this.targetedRotation = positionCalculator.getRobotRotation();
        switchToGoToRotationMode();
    }

    public boolean isRASActivated() { return RASActivation; }

    public void sendCommandsToMotors() {
        if (rotationMode == goToRotationMode) updateRotationalMotorSpeed(dt.seconds());
        if (translationalMode == gotoPositionMode) updateTranslationalMotionUsingEncoder(dt.seconds());
        hardwareDriver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        hardwareDriver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        hardwareDriver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        hardwareDriver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
        // TODO make the robot stick to the rotation it was when pilot not sending commands on rotation
    }

    private void updateRotationalMotorSpeed(double dt) {
        double currentRotation = positionCalculator.getRobotRotation();
        /* according to the angular velocity, predict the future rotation of the robot after velocity debug time */
        double futureRotation = currentRotation + velocityDebugTimeRotation * positionCalculator.getAngularVelocity();

        double rotationalRawError = getActualDifference(currentRotation, targetedRotation);
        double rotationalError = getActualDifference(futureRotation, targetedRotation);

        rotationalIntegration += rotationalRawError * dt;

        rotationalMotion = rotationalError * motorPowerPerRotationDifference + rotationalIntegration * integralCoefficientRotation;
        rotationalMotion = Math.copySign(Math.min(maxRotatingPower, Math.abs(rotationalMotion)), rotationalMotion);
        // rotationalMotion *= -1;
        // System.out.println("rotation:" + Math.toDegrees(positionCalculator.getRobotRotation()) + ";raw error:" + Math.toDegrees(rotationalRawError) + "; error:" + Math.toDegrees(rotationalError) + "; power" + rotationalMotion);
    }

    private void updateTranslationalMotionUsingEncoder(double dt) {
        double[] currentPosition = positionCalculator.getRobotPosition();
        double currentRotation = positionCalculator.getRobotRotation();

        /* according to the robot's translational motion, predict it's future position */
        double[] positionPrediction = new double[2];
        positionPrediction[0] = positionCalculator.getVelocity()[0] * velocityDebugTimeTranslation + currentPosition[0];
        positionPrediction[1] = positionCalculator.getVelocity()[1] * velocityDebugTimeTranslation + currentPosition[1];

        double[] positionRawError = new double[]{xAxleTranslationTarget - currentPosition[0], yAxleTranslationTarget - currentPosition[1]};
        double[] positionError = new double[] {xAxleTranslationTarget - positionPrediction[0], yAxleTranslationTarget- positionPrediction[1]};

        translationalIntegration[0] += dt * positionRawError[0];
        translationalIntegration[1] += dt * positionRawError[1];

        double xAxleMotionToGround;
        double yAxleMotionToGround;
        xAxleMotionToGround = positionError[0] * motorPowerPerEncoderDifference + translationalIntegration[0] * integrationCoefficientTranslation;
        yAxleMotionToGround = positionError[1] * motorPowerPerEncoderDifference + translationalIntegration[1] * integrationCoefficientTranslation;

        xAxleMotionToGround = Math.copySign(
                Math.min(Math.abs(xAxleMotionToGround), maxMotioningPower),
                xAxleMotionToGround
        );
        yAxleMotionToGround = Math.copySign(
                Math.min(Math.abs(yAxleMotionToGround), maxMotioningPower),
                yAxleMotionToGround
        );

        xAxleMotion = (xAxleMotionToGround * Math.cos(currentRotation)) + (yAxleMotionToGround * Math.sin(currentRotation));
        yAxleMotion = (xAxleMotionToGround * Math.sin(currentRotation)) + (yAxleMotionToGround * Math.cos(currentRotation));

        System.out.println("raw error:" + positionRawError[1] + "; error:" + positionError[1] + "; motion(to ground)" + yAxleMotionToGround + "; motion:" + yAxleMotion);
    }

    public static double getActualDifference(double currentRotation, double targetedRotation) {
        while (targetedRotation > Math.PI*2) targetedRotation -= Math.PI*2;
        while (targetedRotation < 0) targetedRotation += Math.PI*2;

        double rawDifference = targetedRotation - currentRotation;
        double absoluteDifference = Math.min(
                Math.abs(rawDifference),
                2*Math.PI - Math.abs(rawDifference));

        if ((rawDifference > Math.PI) || (-Math.PI < rawDifference && rawDifference < 0))  {
            absoluteDifference *= -1;
        }
        return absoluteDifference;
    }
}
