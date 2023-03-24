// TODO write this module as a simple calculator for vertical and horizontal encoders only, in place the real robot position calculator for now
package org.firstinspires.ftc.teamcode.RobotModules;

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

    /** some configurations of the robot TODO:measure these values */
    /** the ratio between the angular velocity(in rad/s) to the difference in the velocity of the two parallel encoders (in encoder value) */
    private final double angularVelocityPerParallelEncoderVelocityDifference = 1;
    /** the ratio between the angular velocity(in rad/s) to the velocity of the third encoder, assuming that the robot's rotating center is still */
    private final double angularVelocityPerThirdEncoderVelocity = 1;

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
    }

    @Override
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {

    }

    @Override
    public void periodic() {

    }
}
