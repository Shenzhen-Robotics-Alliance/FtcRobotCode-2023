package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareDriver;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: AutoStageChassisModule_tmp.java
 *
 * a temporary program that powers the robot's chassis during auto stage
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.27
 * @Version v0.0.1
 * */
public class AutoStageRobotChassis_tmp {
    HardwareDriver hardwareDriver;
    HardwareMap hardwareMap;
    RobotPositionCalculator_tmp positionCalculator;

    /** accept any deviation in position less than 1000 encoder values */
    private static final int positionTolerance = 1000;
    /** the positional deviation when the robot starts slowdown  */
    private static final int positionStartsSlowingDown = 10000;

    /** accept any deviation in rotation less than 10 degrees */
    private static final double rotationTolerance = Math.toRadians(10);
    /** the rotational deviation when the robot starts to decelerate */
    private static final double rotationStartsSlowingDown = Math.toRadians(45);
    /** the rate between the encoder velocity and the angular velocity TODO: adjust it */
    private static final int encoderVelocityPerAngularVelocity = 10000;


    public AutoStageRobotChassis_tmp(HardwareMap hardwareMap, HardwareDriver hardwareDriver, RobotPositionCalculator_tmp positionCalculator) {
        this.hardwareDriver = hardwareDriver;
        this.hardwareMap = hardwareMap;
        this.positionCalculator = positionCalculator;
    }

    public void setRobotRotation(int degrees) {
        this.setRobotRotation(Math.toRadians(degrees));
    }

    public void setRobotPosition(int encoderPositionX, int encoderPositionY) {
        positionCalculator.periodic();
        double startingRotation = positionCalculator.getRobotRotation();

        do {
            setRobotMotion(
                    (encoderPositionX-positionCalculator.getRobotPosition()[0]) / positionStartsSlowingDown,
                    (encoderPositionY-positionCalculator.getRobotPosition()[1]) / positionStartsSlowingDown,
                    (startingRotation - positionCalculator.getRobotRotation()) / rotationTolerance * encoderVelocityPerAngularVelocity
            );
        } while (Math.sqrt(encoderPositionX * encoderPositionX + encoderPositionY * encoderPositionY) > positionTolerance);
    }

    private void setRobotRotation(double radians) {
        do {
            setRobotMotion(0,0,
                    radians - positionCalculator.getRobotRotation() / rotationTolerance * encoderVelocityPerAngularVelocity
                    );
        } while (Math.abs(reformatRotationDifference(radians) - positionCalculator.getRobotRotation()) > rotationTolerance);
    }

    /**
     * powers the motors
     *
     * @param xAxleMotion the objective velocity of the robot in the horizontal direction
     * @param yAxleMotion the objective velocity of the robot in the vertical direction
     * @param rotationalMotion the objective rotational motion of the robot
     */
    private void setRobotMotion(double xAxleMotion, double yAxleMotion, double rotationalMotion) {
        this.hardwareDriver.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hardwareDriver.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hardwareDriver.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.hardwareDriver.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardwareDriver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        hardwareDriver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        hardwareDriver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        hardwareDriver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);
    }

    /**
     * reformat the difference in rotation to avoid bugs
     *
     * @param rawDifference the difference between the objective and actual position
     * @return the amount of radians that the robot needs to rotate, positive for anti-clockwise, ti get to the objective rotation
     */
    private double reformatRotationDifference(double rawDifference) {
        // TODO add explanations
        if (rawDifference > Math.PI) return Math.PI*2 - rawDifference;
        if (rawDifference < -Math.PI) return Math.PI*2 + rawDifference;
        return rawDifference;
    }
}
