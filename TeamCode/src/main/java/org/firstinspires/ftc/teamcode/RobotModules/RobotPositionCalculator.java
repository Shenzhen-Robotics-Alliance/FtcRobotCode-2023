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
public class RobotPositionCalculator extends RobotModule {
    /** the module used to the read the data from encoders */
    private Mini1024EncoderReader encoderReader;

    /** to calculate the difference in time */
    private static final ElapsedTime dt = new ElapsedTime();

    /** some configurations of the robot */
    /** the ratio between the angular velocity(in rad/s) to the difference in the velocity of the two parallel encoders (in encoder value) */
    private static final double angularVelocityPerParallelEncoderVelocityDifference = 10*Math.PI * 2 / 202520.56922511634; // to get this data I rotated the robot 10 rounds on the field and get the difference in encoder value TODO: not accurate, needs an update
    /** the ratio between the angular velocity(in rad/s) to the velocity of the third encoder, assuming that the robot's rotating center is still */
    private static final double angularVelocityPerThirdEncoderVelocity = 31.4 / 69865.0; // to get this data, I also make robot rotate 10 times on the field

    /** stores the robot's current facing, in radian */
    private double robotRotation;
    /** the difference between the encoder value of the two parallel encoders during init */
    private double startingEncoderDifference;
    /** stores the robot's current position, in encoder values */
    private double[] robotPosition = new double[2];
    /** stores the angular velocity of the robot */
    private double angularVelocity = 0;
    /** the velocity of the robot */
    private double[] rawVelocity = new double[2];

    /**
     * construct method of temporary robot position calculator
     */
    public RobotPositionCalculator() {
        super("positionCalculator");
    }

    /**
     * initialize the temporary robot position calculator with given encoder reader module
     * vertical left encoder:1, facing backwards
     * vertical right encoder:2, facing backwards
     * horizontal encoder:3, facing left
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
        if (dependentModules.isEmpty()) throw new NullPointerException (
                "an empty set of dependent modules given to the module<<" + this.getModuleName() + ">> which requires at least one module(s) as dependency"
        );
        if (!dependentModules.containsKey("encoderReader")) throw new NullPointerException(
                "dependency <<" + "encoderReader" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the encoder reader module from the param */
        this.encoderReader = (Mini1024EncoderReader) dependentModules.get("encoderReader");

        this.reset();
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
        this.angularVelocity = getAngularVelocity(encoderReader.getEncoderVelocity(1), encoderReader.getEncoderVelocity(2));

        /** update the robot's current rotation */
        /* take the integral of angular velocity to time */
        // this.robotRotation += angularVelocity * this.dt.seconds();
        this.robotRotation = (encoderReader.getEncoderPosition(2) - encoderReader.getEncoderPosition(1) - startingEncoderDifference) * angularVelocityPerParallelEncoderVelocityDifference; // use the difference between two encoders to determine the heading of the robot
        System.out.println("encoder readings:" + encoderReader.getEncoderPosition(2) + "," + encoderReader.getEncoderPosition(1));
        /* format the rotation value */
        while (this.robotRotation > Math.PI*2) this.robotRotation -= Math.PI*2;
        while (this.robotRotation < 0) this.robotRotation += Math.PI*2;

        /** calculate the robot's velocity, in reference to itself */
        /* calculate the horizontal velocity of the robot by correcting the velocity of the horizontal encoder */
        rawVelocity[0] = correctThirdEncoderVelocity(encoderReader.getEncoderVelocity(3), angularVelocity);
        /*
        * the two vertically installed encoders are identical about the central line of the robot and are parallel to each other
        * therefore, they are influenced equally and reversely by the rotation of the robot
        * so vertical velocity is the mean value of the two encoder values
        * */
        rawVelocity[1] = (encoderReader.getEncoderVelocity(1) + encoderReader.getEncoderVelocity(2)) / 2;

        /** do an integral of the actual velocity to time towards calculate the robot's position */
        this.robotPosition[0] += getActualVelocity(rawVelocity, robotRotation)[0] * dt.seconds();
        this.robotPosition[1] += getActualVelocity(rawVelocity, robotRotation)[1] * dt.seconds();

        // System.out.println(     "robot rotation:" + robotRotation);
        // System.out.println("raw velocity:" + rawVelocity[0] + ", " + rawVelocity[1]); // TODO the problem originated from the raw velocity
        // System.out.println("actual velocity: " + getActualVelocity(rawVelocity, robotRotation)[0] + ", " + getActualVelocity(rawVelocity, robotRotation)[1]);

        this.dt.reset();
    }

    /** force the encoder reader to update the readings */
    public void forceUpdateEncoderValue() { encoderReader.periodic(); }

    /**
     * determines the angular velocity of the robot using the difference between the velocity of the two parallel encoders
     *
     * @param parallelEncoder1Velocity the current velocity of the first parallel encoder
     * @param parallelEncoder2Velocity the current velocity of the second parallel encoder
     * @return the calculated angular velocity of the robot, in rad/s
     */
    private static double getAngularVelocity(double parallelEncoder1Velocity, double parallelEncoder2Velocity) {
        /* calculate the difference between the velocity of the two parallel encoders */
        double velocityDifference = parallelEncoder2Velocity - parallelEncoder1Velocity;

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

    /**
     * calculate, using the raw velocity captured by the encoders, the actual velocity vector of the robot, in reference to ground
     *
     * @param rawVelocity the raw velocity captured by the encoders
     * @param robotHeading the robot's current facing, obtained from the imu module or calculated form the encoders
     * @return the actual velocity, according to the field, in encoder values
     */
    private static double[] getActualVelocity(double[] rawVelocity, double robotHeading) {
        double[] actualVelocity = new double[2];

        /* the effect on the actual horizontal(in reference to the field) component of the velocity by the raw horizontal(according to the robot) part of the velocity */
        actualVelocity[0] += rawVelocity[0] * Math.cos(robotHeading);
        /* the effect on the actual horizontal(in reference to the field) component of the velocity by the raw vertical(according to the robot) part of the velocity */
        actualVelocity[0] += rawVelocity[1] * Math.cos(robotHeading + Math.toRadians(90));
        /* the effect on the actual vertical(in reference to the field) component of the velocity by the raw horizontal(according to the robot) part of the velocity */
        actualVelocity[1] += rawVelocity[0] * Math.sin(robotHeading);
        /* the effect on the actual vertical(in reference to the field) component of the velocity by the raw vertical(according to the robot) part of the velocity */
        actualVelocity[1] += rawVelocity[1] * Math.sin(robotHeading + Math.toRadians(90));

        return actualVelocity;
    }

    public double[] getRobotPosition() {
        return robotPosition;
    }

    public double getRobotRotation() {
        return robotRotation;
    }

    public double[] getRawVelocity() { return rawVelocity; }

    public double getAngularVelocity() { return angularVelocity; }

    public double[] getVelocity() { return getActualVelocity(rawVelocity, robotRotation); }

    /** reset the position calculator to initial stat */
    public void reset() {
        /* start the timer */
        this.dt.reset();

        /* set the robot to be in zero position */
        robotPosition[0] = 0;
        robotPosition[1] = 0;
        robotRotation = 0;

        rawVelocity[0] = 0;
        rawVelocity[1] = 0;

        startingEncoderDifference = encoderReader.getEncoderPosition(2) - encoderReader.getEncoderPosition(1);
    }
}
