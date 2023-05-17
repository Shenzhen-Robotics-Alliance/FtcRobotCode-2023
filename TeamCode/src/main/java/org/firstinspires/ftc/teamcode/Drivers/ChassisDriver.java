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
    private final double velocityDebugTime = 0.05;
    private final double integralCoefficient = 0; // not needed yet

    private HardwareDriver hardwareDriver;
    private RobotPositionCalculator positionCalculator;

    private double xAxleMotion = 0;
    private double yAxleMotion = 0;
    private double rotationalMotion = 0;

    private double xAxleTranslationTarget = 0;

    private double yAxleTranslationTarget = 0;
    private double targetedRotation = 0;
    private boolean RASActivation = false;

    private final int goToRotationMode = 1;
    private final int manualMode = 0;
    private int rotationMode = manualMode;
    private final int gotoPositionMode = 2;
    private final int driverUsingTOFSensorMode = 3;
    private int translationalMode = manualMode;

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
        switchToManualMode();
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

    public void switchToManualMode() { rotationMode = manualMode; }

    private void switchToGoToPositionMode() { rotationMode = goToRotationMode; }

    public boolean isRASActivated() {
        return RASActivation;
    }

    public void sendCommandsToMotors() {
        if (rotationMode == goToRotationMode) updateRotationalMotorSpeed();
        if (translationalMode == driverUsingTOFSensorMode)
        hardwareDriver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        hardwareDriver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        hardwareDriver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        hardwareDriver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
        // TODO make the robot stick to the rotation it was when pilot not sending commands on rotation
    }

    private void updateRotationalMotorSpeed() {
        double currentRotation = positionCalculator.getRobotRotation();
        /* according to the angular velocity, predict the future rotation of the robot after velocity debug time */
        double futureRotation = currentRotation + velocityDebugTime * positionCalculator.getAngularVelocity();

        double rotationalRawError = getActualDifference(currentRotation, targetedRotation);
        double rotationalError = getActualDifference(futureRotation, targetedRotation);

        integration += rotationalRawError * dt.seconds();

        rotationalMotion = rotationalError * motorPowerPerRotationDifference + integration * integralCoefficient;
        rotationalMotion = Math.copySign(Math.min(maxRotatingPower, Math.abs(rotationalMotion)), rotationalMotion);
        // rotationalMotion *= -1;
        System.out.println("rotation:" + Math.toDegrees(positionCalculator.getRobotRotation()) + ";raw error:" + Math.toDegrees(rotationalRawError) + "; error:" + Math.toDegrees(rotationalError) + "; power" + rotationalMotion);
        dt.reset();
    }

    private void updateTranslationalMotionUsingTOFDistanceSensor(double tofDistanceSensorReading) {

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
