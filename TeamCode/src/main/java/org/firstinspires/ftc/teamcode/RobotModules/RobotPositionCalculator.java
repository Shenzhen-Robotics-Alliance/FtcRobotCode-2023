package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: RobotPositionCalculator.java
 *
 * the module that calculates the robot's position according to the data from the encoders
 * TODO complete this method using linear algebra
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.17
 * @Version v0.0.1
 */
public class RobotPositionCalculator extends RobotModule {
    /** some configurations of the encoders TODO: edit these settings according to the robot */

    /** the difference in amount of encoder values that the first encoder senses when the robot moves every 1 millimeter */
    private final double firstEncoderSensitivity = 1;
    /** the place where the first encoder is installed, in reference to the center of the robot and in millimeters */
    private final double[] firstEncoderInstallationBias = {0, 0};
    /** the facing of the first encoder, in radian */
    private final double firstEncoderInstallationFacing = 0;

    /** the difference in amount of encoder values that the second encoder senses when the robot moves every 1 millimeter */
    private final double secondEncoderSensitivity = 1;
    /** the place where the second encoder is installed, in reference to the center of the robot and in millimeters */
    private final double[] secondEncoderInstallationBias = {0, 0};
    /** the facing of the second encoder, in radian */
    private final double secondEncoderInstallationFacing = 0;

    /** whether to use the third encoder, or to use the imu to sense the direction instead */
    private boolean useThirdEncoderInsteadOfIMU = false;
    /** the difference in amount of encoder values that the third encoder senses when the robot moves every 1 millimeter */
    private final double thirdEncoderSensitivity = 1;
    /** the place where the second encoder is installed, in reference to the center of the robot and in millimeters */
    private final double[] thirdEncoderInstallationBias = {0, 0};
    /** the facing of the second encoder, in radian */
    private final double thirdEncoderInstallationFacing = 0;


    /** the module that reads and analyze the encoder data */
    private Mini1024EncoderReader encoderReader;

    /** the timer to store the time elapsed between two periods */
    private final ElapsedTime dt = new ElapsedTime();



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
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances,
            boolean useThirdEncoderInsteadOfIMU
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

        /* set whether to use the third encoder */
        this.useThirdEncoderInsteadOfIMU = useThirdEncoderInsteadOfIMU;
    }

    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        init(dependentModules, dependentInstances, false);
    }


    /**
     * updates a dependent instance of the module
     *
     * @Deprecated the robot position calculator currently does not support updating instances after init
     */
    @Override @Deprecated
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}


    /** update the position according to readings of the imu and the encoders */
    @Override
    public void periodic() {
        if (useThirdEncoderInsteadOfIMU) updatePositionUsingThirdEncoder(dt.seconds());
        else updatePositionUsingIMU(dt.seconds());
        dt.reset();
    }

    /**
     * calculates the position using the imu and the two encoders
     *
     * @param dt the change in time between two periods
     */
    private void updatePositionUsingIMU(double dt) {

    }

    /**
     * calculate the position using all three encoders
     *
     * @param dt the change in time between two periods
     */
    private void updatePositionUsingThirdEncoder(double dt) {

    }
}

/**
 * represents a matrix, whether 2 by 2 or 2 by 1
 * for easier calculation
 */
abstract class Matrix {
    /** the size of the matrix, in the form of {columns, rows}*/
    protected short[] size = new short[2];

    /** the contend of the matrix */
    protected double[][] contend;

    // TODO write the calculations between matrices
}

/** representing a vector in 2d space */
class Vector2d extends Matrix {
    public Vector2d(double x, double y) {
        this.size = new short[] {1, 2};
        this.contend = new double[1][2];
    }
}

/** represents a linear transformation in 2d space */
class linearTransform {}