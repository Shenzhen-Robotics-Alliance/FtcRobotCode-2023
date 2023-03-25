// TODO write this module as a simple calculator for vertical and horizontal encoders only, in place the real robot position calculator for now
package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: RobotPositionCalculator_tmp.java
 *
 * a rough calculator of the robot's position using two vertically facing encoders and one horizontally facing encoders
 * the encoders are identical in all ways except for the place they are installed
 * the vertical encoders are identical according to the central line of the robot and are equally distanced from the rotating center of the chassis
 *
 * my method to navigate:
 *  - the difference between the two vertical encoders, which are parallel, is always proportion to rotation of the robot, despite how the robot moves vertically or horizontally
 *  - use angular velocity calculated from the process above can be used to erase the effect of rotation
 *  - simply subtract the velocity reading of an encoder by it's linear velocity when the robot is rotating at that angular velocity calculated above
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.24
 * @Version v0.0.0
 */
public class RobotPositionCalculator_tmp extends RobotModule {
    /** the module used to the read the data from encoders */
    private Mini1024EncoderReader encoderReader;

    /** to calculate the difference in time */
    private static final ElapsedTime dt = new ElapsedTime();

    /** some configurations of the robot TODO:measure these values */
    /** the ratio between the angular velocity(in rad/s) to the difference in the velocity of the two parallel encoders (in encoder value) */
    private static final double angularVelocityPerParallelEncoderVelocityDifference = 1;
    /** the ratio between the angular velocity(in rad/s) to the velocity of the third encoder, assuming that the robot's rotating center is still */
    private static final double angularVelocityPerThirdEncoderVelocity = 1;

    /** stores the robot's current facing, in radian */
    private double robotRotation;
    /** stores the robot's current position, in encoder values */
    private double[] robotPosition;

    /**
     * construct method of temporary robot position calculator
     */
    public RobotPositionCalculator_tmp() {
        super("temporaryPositionCalculator");
    }

    /**
     * initialize the temporary robot position calculator with given encoder reader module
     *
     * @param dependentModules the following modules are required for robot position calculator
     *                         "encoderReader": Mini1024EncoderReader, the module that reads and processes the encoder value
     * @param dependentInstances null should be given as this module does not dependent on any instance
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        /* throw out an error if encoder reader module not given */
        if (dependentInstances.isEmpty()) throw new NullPointerException (
                "an empty set of dependent modules given to the module<<" + this.getModuleName() + ">> which requires at least one module(s) as dependency"
        );
        if (!dependentModules.containsKey("encoderReader")) throw new NullPointerException(
                "dependency <<" + "encoderReader" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the encoder reader module from the param */
        this.encoderReader = (Mini1024EncoderReader) dependentModules.get("encoderReader");

        /* start the timer */
        this.dt.reset();
    }

    /**
     * updates an instance of this module
     *
     * @Deprecated the robot position calculator does not need any instances in the first place
     */
    @Override @Deprecated public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    /** updates the robot's current position by taking the integral of the calculated robot velocity over all times */
    @Override
    public void periodic() {
        /** calculate the angular velocity of the robot */
        double angularVelocity = getAngularVelocity(encoderReader.getEncoderVelocity(1), encoderReader.getEncoderVelocity(2));

        /** update the robot's current rotation */
        /* take the integral of angular velocity to time */
        this.robotRotation += angularVelocity * dt.seconds();
        /* format the rotation value */
        while (this.robotRotation > Math.PI*2) this.robotRotation -= Math.PI*2;
        while (this.robotRotation < 0) this.robotRotation += Math.PI*2;

        /** calculate the robot's velocity, in reference to itself */
        double[] rawVelocity = new double[2];
        /* calculate the horizontal velocity of the robot by correcting the velocity of the horizontal encoder */
        rawVelocity[0] = correctThirdEncoderVelocity(angularVelocity, encoderReader.getEncoderVelocity(3));
        /*
        * the two vertically installed encoders are identical about the central line of the robot and are parallel to each other
        * therefore, they are influenced equally and reversely by the rotation of the robot
        * so vertical velocity is the mean value of the two encoder values
        * */
        rawVelocity[1] = (encoderReader.getEncoderVelocity(1) + encoderReader.getEncoderVelocity(2)) / 2;
    }

    /**
     * determines the angular velocity of the robot using the difference between the velocity of the two parallel encoders
     *
     * @param parallelEncoder1Velocity the current velocity of the first parallel encoder
     * @param parallelEncoder2Velocity the current velocity of the second parallel encoder
     * @return the calculated angular velocity of the robot, in rad/s
     */
    private static double getAngularVelocity(double parallelEncoder1Velocity, double parallelEncoder2Velocity) {
        /* calculate the difference between the velocity of the two parallel encoders */
        double velocityDifference = parallelEncoder1Velocity - parallelEncoder2Velocity;

        /*
        * the two encoders are installed vertically, parallel to each other, and identical about the central line of the robot
        * so the difference between the two encoders are always proportional to the angular velocity of the robot
        * */
        double robotRotation = velocityDifference * angularVelocityPerParallelEncoderVelocityDifference;

        return robotRotation;
    }

    private static double correctThirdEncoderVelocity(double thirdEncoderRawVelocity, double angularVelocity) {
        /*
        * the third encoder is installed horizontally
        * so the linear velocity of the third horizontal encoder, assuming the robot is still can be calculated
        * though the third encoder not installed at the central of the robot
        * but the angle between the encoder's facing and the perpendicular line of the line that connects the third encoder and the central is fixed
        * so the influence on the reading of the third encoder is still always proportional to the angular velocity of the robot
        * */
        double thirdEncoderLinearVelocity = angularVelocity / angularVelocityPerThirdEncoderVelocity;

        /*
        * to calculate the robot's linear movement
        * simply subtract the actual value by the rotation velocity
        * */
        double thirdEncoderActualVelocity = thirdEncoderRawVelocity - thirdEncoderLinearVelocity;

        return thirdEncoderActualVelocity;
    }
}
