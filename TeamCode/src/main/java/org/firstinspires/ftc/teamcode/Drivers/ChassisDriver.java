package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModules.RobotPositionCalculator;

public class ChassisDriver {
    /* private final double maxPower = 0.6;
    private final double encoderDistanceStartDecelerate = 15000;
    private final double motorPowerPerEncoderValueError = (maxPower / encoderDistanceStartDecelerate); */
    private final double maxRotatingPower = 0.5;
    private final double rotationDifferenceStartDecelerate = Math.toRadians(40);
    private final double motorPowerPerRotationDifference = (maxRotatingPower / rotationDifferenceStartDecelerate);
    private final double velocityDebugTime = 0.2;
    private final double integralCoefficient = 0.01;

    private HardwareDriver hardwareDriver;
    private RobotPositionCalculator positionCalculator;

    private double xAxleMotion = 0;
    private double yAxleMotion = 0;
    private double rotationalMotion = 0;

    private double xAxleTranslationTarget = 0;

    private double yAxleTranslationTarget = 0;
    private double targetedRotation = 0;
    private boolean RASActivation = false;

    private final int goToPositionMode = 1;
    private final int manualMode = 0;
    private int mode = manualMode;

    private double integration;

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
        switchToGoToPositionMode();
    }

    /** in radian */
    public void setTargetedRotation(double targetedRotation) {
        this.targetedRotation = targetedRotation;
        switchToGoToPositionMode();
    }

    public void setRotationalMotion(double rotationalMotion) {
        this.rotationalMotion = rotationalMotion;
        dt.reset();
        integration = 0;
        sendCommandsToMotors();
    }

    public void pilotInterruption() {
        RASActivation = false;
    }

    public void newAimStarted() {
        RASActivation = true;
    }

    public void aimStopped() {RASActivation = false; }

    public void switchToManualMode() { mode = manualMode; }

    private void switchToGoToPositionMode() { mode = goToPositionMode; }

    public boolean isRASActivated() {
        return RASActivation;
    }

    public void sendCommandsToMotors() {
        if (mode == goToPositionMode) updateMotorSpeed();
        hardwareDriver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        hardwareDriver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        hardwareDriver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        hardwareDriver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
    }

    private void updateMotorSpeed() {
        // TODO: translation is not considered yet as it not needed

        double currentRotation = positionCalculator.getRobotRotation();
        /* according to the angular velocity, predict the future rotation of the robot after velocity debug time */
        double futureRotation = currentRotation + velocityDebugTime * positionCalculator.getAngularVelocity();

        double rotationalRawError = targetedRotation - currentRotation;
        double rotationalError = targetedRotation - futureRotation;

        integration += rotationalRawError * dt.seconds();

        rotationalMotion = rotationalError * motorPowerPerRotationDifference + integration * integralCoefficient;
        rotationalMotion = Math.copySign(Math.min(maxRotatingPower, Math.abs(rotationalMotion)), rotationalMotion);
        // rotationalMotion *= -1;
        System.out.println("raw error:" + rotationalRawError);
        dt.reset();
    }
}
