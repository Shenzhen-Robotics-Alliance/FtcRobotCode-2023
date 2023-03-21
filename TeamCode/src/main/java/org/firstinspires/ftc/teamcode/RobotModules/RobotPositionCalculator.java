package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: RobotPositionCalculator.java
 *
 * the module that calculates the robot's position according to the data from the encoders
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.17
 * @Version v0.0.1
 */
public class RobotPositionCalculator extends RobotModule {
    /** the module that reads and analyze the encoder data */
    private Mini1024EncoderReader encoderReader;

    /**
     * initialize the position calculator
     */
    public RobotPositionCalculator() {
        super("robotPositionCalculator");
    }

    /**
     * initialize the position calculator by offering the connection to mini1024 encoder reader module
     *
     * @param dependentModules the related modules needed by this position calculator
     *                         "encoderReader" : Mini1024EncoderReader, the module that reads the encoder's value
     * @param dependentInstances null would be OK as this module does not need any instance
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        /* throw out an error if the dependent module is given an empty map */
        if (dependentModules.isEmpty()) throw new NullPointerException(
                "an empty map of dependent modules given to this module, which requires at least one modular dependencies"
        );

        /* get the dependent modules from the param */
        if (! dependentModules.containsKey("encoderReader")) throw new NullPointerException(
                "dependent module not given: " + "encoderReader"
        );
        this.encoderReader = (Mini1024EncoderReader) dependentModules.get("encoderReader");
    }

    @Override
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {

    }

    @Override
    public void periodic() throws InterruptedException {

    }
}