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
    private static final int positionTolerance = 500;
    /** the positional deviation when the robot starts slowdown  */
    private static final int positionStartsSlowingDown = 6000;

    /** accept any deviation in rotation less than 10 degrees */
    private static final double rotationTolerance = Math.toRadians(5);
    /** the rotational deviation when the robot starts to decelerate */
    private static final double rotationStartsSlowingDown = Math.toRadians(45);
    /** minimum power to make the robot move */
    private static final double minMovingMotorPower = 0.25;
    /** the amount of time that the robot needs to slow down */
    private static final double timeForSlowDown = 0.3;
    /** maximum power during auto stage */
    private static final double maxMovingMotorPower = 0.45;
    /** the power needed to rotate the robot is slightly smaller than that needed to move it */
    private static final double rotationPowerFactor = -0.6;
    /** whether to flip the x-axis for left side operation */
    private static final double xAxisPositionCorrectionFactor = 1;


    public AutoStageRobotChassis_tmp(HardwareMap hardwareMap, HardwareDriver hardwareDriver, RobotPositionCalculator_tmp positionCalculator) {
        this.hardwareDriver = hardwareDriver;
        this.hardwareMap = hardwareMap;
        this.positionCalculator = positionCalculator;
    }

    public void setRobotRotation(int degrees) {
        this.setRobotRotation(Math.toRadians(degrees));
    }

    public void setRobotPosition(int encoderPositionX, int encoderPositionY, double startingRotation) {
        encoderPositionX *= xAxisPositionCorrectionFactor;

        /** whether the process is finished */
        boolean completed = false;

        do {
            /** update sensor readings */
            positionCalculator.forceUpdateEncoderValue();
            positionCalculator.periodic();

            /** calculate bias between the current and the starting rotation */
            double rotationalDifference = reformatRotationDifference(startingRotation - positionCalculator.getRobotRotation());
            double rotationCorrectionMotorSpeed = Math.copySign(RobotChassis.linearMap(
                    rotationTolerance,rotationStartsSlowingDown,0,maxMovingMotorPower,
                    Math.abs(rotationalDifference)
            ) , rotationalDifference);

            /** the bias between the current and the targeted position in the x-axis, in reference to the field */
            double xAxisFieldDifference = encoderPositionX - positionCalculator.getRobotPosition()[0];
            double yAxisFieldDifference = encoderPositionY - positionCalculator.getRobotPosition()[1];

            /** take in consider the motion of the robot */
            xAxisFieldDifference -= positionCalculator.getVelocity()[0] * timeForSlowDown;
            yAxisFieldDifference -= positionCalculator.getVelocity()[1] * timeForSlowDown;

            /** calculates the velocity needed in reference to the ground, do a linear map to get the motor speed */
            double xAxisFieldVelocity, yAxisFieldVelocity;
            if (Math.abs(yAxisFieldDifference) > positionTolerance*2 && Math.abs(xAxisFieldDifference) < positionTolerance*2) {
                /* if the robot is already moving along y axis, we don't need minimum power to keep it moving in the x-drection*/
                xAxisFieldVelocity = Math.copySign(
                        RobotChassis.linearMap(
                                positionTolerance,
                                positionStartsSlowingDown,
                                0,
                                maxMovingMotorPower,
                                Math.abs(xAxisFieldDifference)
                        ), xAxisFieldDifference);
            } else {
                /* else then, give it a minimum velocity in x-axis */
                xAxisFieldVelocity = Math.copySign(
                        RobotChassis.linearMap(
                                positionTolerance,
                                positionStartsSlowingDown,
                                minMovingMotorPower,
                                maxMovingMotorPower,
                                Math.abs(xAxisFieldDifference)
                        ), xAxisFieldDifference);
            }
            if (Math.abs(xAxisFieldDifference) > positionTolerance*2 && Math.abs(yAxisFieldDifference) < positionTolerance*2) {
                yAxisFieldVelocity = Math.copySign(
                        RobotChassis.linearMap(
                                positionTolerance,
                                positionStartsSlowingDown,
                                0,
                                maxMovingMotorPower,
                                Math.abs(yAxisFieldDifference)
                        ), yAxisFieldDifference);
            } else {
                yAxisFieldVelocity = Math.copySign(
                        RobotChassis.linearMap(
                                positionTolerance,
                                positionStartsSlowingDown,
                                minMovingMotorPower,
                                maxMovingMotorPower,
                                Math.abs(yAxisFieldDifference)
                        ), yAxisFieldDifference);
            }
            System.out.println(rotationalDifference + ", " + rotationCorrectionMotorSpeed);

            /** determine, according to the robot's heading the velocity that the robot needs to move to achieve the field velocity */
            double xAxisAbsoluteVelocity = xAxisFieldVelocity * Math.cos(positionCalculator.getRobotRotation()) // the effect of x-axis field velocity on the robot's x-axis velocity
                    + yAxisFieldVelocity * Math.sin(positionCalculator.getRobotRotation()); // the effect of y-axis field velocity on the robot's x-axis velocity
            double yAxisAbsoluteVelocity = xAxisFieldVelocity * Math.sin(positionCalculator.getRobotRotation()) // the effect of x-axis field velocity on the robot's y-axis velocity
                    + yAxisFieldVelocity * Math.cos(positionCalculator.getRobotRotation()); // the effect of y-axis field velocity on the robot's y-axis velocity

            /** set the motors to run, correct the motor speed for rotation and scale it down a bit as we don't want it to shake */
            setRobotMotion(xAxisAbsoluteVelocity, yAxisAbsoluteVelocity, rotationCorrectionMotorSpeed * rotationPowerFactor * 0.8);
            /** jump out of the loop when the robot reaches the targeted area */
            completed = Math.sqrt(xAxisFieldDifference * xAxisFieldDifference + yAxisFieldDifference * yAxisFieldDifference) < positionTolerance;
            completed = Math.abs(xAxisFieldDifference) < positionTolerance*2 && Math.abs(yAxisFieldDifference) < positionTolerance*2;
        } while (!completed);
        /* set the motors to stop */
        hardwareDriver.leftFront.setVelocity(0);
        hardwareDriver.leftRear.setVelocity(0);
        hardwareDriver.rightFront.setVelocity(0);
        hardwareDriver.rightRear.setVelocity(0);

        /* wait until the robot is completely still */
        while (Math.abs(positionCalculator.getRawVelocity()[0]) > 200 || Math.abs(positionCalculator.getRawVelocity()[1]) > 200) {
            positionCalculator.forceUpdateEncoderValue();
            positionCalculator.periodic();
        }
    }

    public void setRobotPosition(int encoderPositionX, int encoderPositionY) {
        /** get the rotation by the start of the move */
        double startingRotation = positionCalculator.getRobotRotation();

        setRobotPosition(encoderPositionX, encoderPositionY, startingRotation);
    }

    private void setRobotRotation(double radians) {
        do {
            /* update sensor readings */
            positionCalculator.forceUpdateEncoderValue();
            positionCalculator.periodic();

            /* calculate bias between the current and the targeted rotation */
            double rotationalDifference = reformatRotationDifference(radians - positionCalculator.getRobotRotation());

            /* do a linear map do determine how much motor power is used to rotate the robot  */
            double rotationalPower = Math.copySign(RobotChassis.linearMap(
                    rotationTolerance,rotationStartsSlowingDown,minMovingMotorPower,maxMovingMotorPower,
                    Math.abs(rotationalDifference)
            ) , rotationalDifference);

            /* set the power of the motors */
            setRobotMotion(0,0,rotationalPower * rotationPowerFactor);

            System.out.println(radians - positionCalculator.getRobotRotation() + ", " + rotationalDifference);
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
    public static double reformatRotationDifference(double rawDifference) {
        /* if the rotational difference is greater than 180 degree, and that the objective is in the clockwise direction of the current, go the other way around(turn counter-clockwise) */
        if (rawDifference > Math.PI) return Math.PI*2 - rawDifference;
        /* if the rotation difference is greater than 180, and that the objective is in counter-clockwise, go clockwise */
        if (rawDifference < -Math.PI) return Math.PI*2 + rawDifference;
        /* if the rotation difference is no greater than 180, just turn to the objective directly */
        return rawDifference;
    }
}
