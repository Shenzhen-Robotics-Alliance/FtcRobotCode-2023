package org.firstinspires.ftc.teamcode.RobotModules;

import android.os.SystemClock;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotModule;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Mini1024EncoderModule
 *
 * the module that reads the angular position, velocity and acceleration of the encoders
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.16
 * @Version v0.0.3
*/
public class Mini1024EncoderReader extends RobotModule {
    /** the operable instance of the three encoders */
    private DcMotorEx encoder1, encoder2, encoder3;

    /** the current position of the encoders, updated every period of the run loop */
    private double encoder1Position, encoder2Position, encoder3Position;
    /** the current velocity of the encoders, updated every period of the run loop, determined using the change in position and the difference in time */
    private double encoder1Velocity, encoder2Velocity, encoder3Velocity;
    /** the current position of the encoders, updated every period of the run loop, determined using the change in velocity and the difference in time */
    private double encoder1Acceleration, encoder2Acceleration, encoder3Acceleration;

    /** the starting position of the encoders, updated when the encoders are requested to calibrate */
    private double encoder1StartingPosition, encoder2StartingPosition, encoder3StartingPosition;

    /** whether to read the data from encoder3 */
    private boolean useEncoder3;

    /** the correction factor of the first encoders */
    private double encoder1CorrectionFactor = 1;
    /** the correction factor of the second encoders */
    private double encoder2CorrectionFactor = 1;
    /** the correction factor of the third encoders */
    private double encoder3CorrectionFactor = 1;

    /** the difference in time between two adjacent periods */
    private ElapsedTime dt = new ElapsedTime();

    /**
     * construct method of the reader of mini1024 encoder
     */
    public Mini1024EncoderReader() {
        super("encoderReader");
    }

    /**
     * initialize the encoders
     *
     * @param dependentModules: not needed
     * @param dependentInstances
     *                          DcMotorEx "encoder-1-instance", "encoder-2-instance", "encoder-3-instance3":
     *                          the instance of the three encoders
     * @param useEncoder3 whether to read from encoder 3
     * @param reverseEncoder1  whether to inverse encoder1's value
     * @param reverseEncoder2  whether to inverse encoder2's value
     * @param reverseEncoder3  whether to inverse encoder3's value
     */
    public void init (
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances,
            boolean useEncoder3,
            boolean reverseEncoder1,
            boolean reverseEncoder2,
            boolean reverseEncoder3
    ) throws NullPointerException {
        /* get the three encoders from the args */
        encoder1 = (DcMotorEx) dependentInstances.get("encoder-1-instance");
        encoder2 = (DcMotorEx) dependentInstances.get("encoder-2-instance");

        /* the third encoder is optional */
        this.useEncoder3 = useEncoder3;
        if (useEncoder3) encoder3 = (DcMotorEx) dependentInstances.get("encoder-3-instance");

        /* reverse the value of the encoders if asked */
        if (reverseEncoder1) encoder1CorrectionFactor = -1;
        if (reverseEncoder2) encoder2CorrectionFactor = -1;
        if (reverseEncoder3) encoder3CorrectionFactor = -1;

        /* initialize the time */
        dt.reset();

        /* initialize the encoders */
        calibrateEncoder(1);
        calibrateEncoder(2);
        calibrateEncoder(3);

        System.out.println(encoder2StartingPosition + ", " + encoder2Position);
    }

    /**
     * initialize the encoders
     *
     * @param dependentModules: not needed
     * @param dependentInstances
     *                          DcMotorEx "encoder-1-instance", "encoder-2-instance", "encoder-3-instance3":
     *                          the instance of the three encoders
     */
    @Override
    public void init (
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        this.init(dependentModules, dependentInstances, true, false, false , false);
    }

    /**
     * @Deprecated the encoders should not be changed after initialization
     */
    @Override
    @Deprecated
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {
        throw new NullPointerException("cannot modify the encoders once the initialization is over");
    }

    /**
     * called every period of run loop
     * get the position, and use it to calculate the velocity and acceleration
     */
    @Override
    public void periodic() {
        /** position */
        /* get the current position of the three encoders */
        double encoder1CurrentPosition = encoder1.getCurrentPosition();
        double encoder2CurrentPosition = encoder2.getCurrentPosition();
        double encoder3CurrentPosition = 0; if (useEncoder3) encoder3CurrentPosition = encoder3.getCurrentPosition();

        /* calculate the change in each of these positions */
        double encoder1PositionDifference = encoder1CurrentPosition - encoder1Position;
        double encoder2PositionDifference = encoder2CurrentPosition - encoder2Position;
        double encoder3PositionDifference = encoder3CurrentPosition - encoder3Position;

        /* the older version of the positions are no longer needed, refresh them */
        encoder1Position = encoder1CurrentPosition;
        encoder2Position = encoder2CurrentPosition;
        encoder3Position = encoder3CurrentPosition;

        /** velocity */
        /* calculate the current velocity */
        double encoder1CurrentVelocity = encoder1PositionDifference / dt.seconds();
        double encoder2CurrentVelocity = encoder2PositionDifference / dt.seconds();
        double encoder3CurrentVelocity = encoder3PositionDifference / dt.seconds();

        /* calculate the chang in velocities */
        double encoder1VelocityDifference = encoder1CurrentVelocity - encoder1Velocity;
        double encoder2VelocityDifference = encoder2CurrentVelocity - encoder2Velocity;
        double encoder3VelocityDifference = encoder3CurrentVelocity - encoder3Velocity;

        /* the older version of the velocities are used, refresh them */
        encoder1Velocity = encoder1CurrentVelocity;
        encoder2Velocity = encoder2CurrentVelocity;
        encoder3Velocity = encoder3CurrentVelocity;


        /** acceleration */
        /* calculate the acceleration */
        encoder1Acceleration = encoder1VelocityDifference / dt.seconds();
        encoder2Acceleration = encoder2VelocityDifference / dt.seconds();
        encoder3Acceleration = encoder3VelocityDifference / dt.seconds();


        /** reset the timer */
        dt.reset();
    }

    /**
     * set the starting position of the encoders
     *
     * @param id: the id for the encoder, 1 2 or 3
     * @param
     * @throws IndexOutOfBoundsException if an none-exist encoder is selected
     */
    private void setEncoderStartingPosition(int id, double startingPosition) {
        switch (id) {
            case 1: {
                encoder1StartingPosition = startingPosition;
                break;
            }
            case 2: {
                encoder2StartingPosition = startingPosition;
                break;
            }
            case 3: {
                encoder3StartingPosition = startingPosition;
                break;
            }
            default: {
                throw new IndexOutOfBoundsException();
            }
        }
    }

    /**
     * get the current position of an encoder
     *
     * @param id: the id of the encoder, 1 2 or 3
     * @return position: the current position of the selected encoder
     * @throws IndexOutOfBoundsException if an none-exist encoder is selected
     */
    public double getEncoderPosition(int id)
            throws IndexOutOfBoundsException {
        switch (id) {
            /* return the current position of the selected encoder, minus the starting position of which */
            case 1: {
                return encoder1Position - encoder1StartingPosition;
            }
            case 2: {
                return encoder2Position - encoder2StartingPosition;
            }
            case 3: {
                return encoder3Position - encoder3StartingPosition;
            }
            default: {
                IndexOutOfBoundsException indexOutOfBoundsException = new IndexOutOfBoundsException();
                throw indexOutOfBoundsException;
            }
        }
    }

    /**
     * get the velocity of an encoder
     * calculated using the difference in time and the change in encoder position over the last period
     *
     * @param id the id of the wanted encoder, 1 2 or 3
     * @return velocity: the current velocity of the selected encoder
     * @throws IndexOutOfBoundsException if an none-exist encoder is selected
     */
    public double getEncoderVelocity(int id)
            throws IndexOutOfBoundsException {
        switch (id) {
            case 1: {
                return encoder1Velocity;
            }
            case 2: {
                return encoder2Velocity;
            }
            case 3: {
                return encoder3Velocity;
            }
            default: {
                IndexOutOfBoundsException indexOutOfBoundsException = new IndexOutOfBoundsException();
                throw indexOutOfBoundsException;
            }
        }
    }

    /**
     * get the acceleration of an encoder
     * calculated using the difference in time and the change in encoder position over the last period
     *
     * @param id: the id of the encoder, 1 2 or 3
     * @return acceleration: the current acceleration of the selected encoder
     * @throws IndexOutOfBoundsException if an none-exist encoder is selected
     */
    public double getEncoderAcceleration(int id)
            throws IndexOutOfBoundsException {
        switch (id) {
            case 1: {
                return encoder1Acceleration;
            }
            case 2: {
                return encoder2Acceleration;
            }
            case 3: {
                return encoder3Acceleration;
            }
            default: {
                IndexOutOfBoundsException indexOutOfBoundsException = new IndexOutOfBoundsException();
                throw indexOutOfBoundsException;
            }
        }
    }

    /**
     * calibrate an encoder
     * set the reference of zero position of the selected encoder to be its current position
     *
     * @param id the id of the desired encoder , 1 2 or 3
     * @throws IndexOutOfBoundsException if an none-exist encoder is selected
     */
    public void calibrateEncoder(int id) throws IndexOutOfBoundsException {
        /* wait for the hardware initialization to complete */
        // while (getEncoderPosition(id) == 0) SystemClock.sleep(50);
        /* set the starting position */
        System.out.println("encoderStartingPosition:" + getEncoderPosition(id));
        double selectedEncoderCurrentPosition = getEncoderPosition(id);
        setEncoderStartingPosition(id, selectedEncoderCurrentPosition);

        /* re-initialize all the variables */
        encoder1Position = encoder2Position = encoder3Position = 0;
        encoder1Velocity = encoder2Velocity = encoder3Velocity = 0;
        encoder1Acceleration = encoder2Acceleration = encoder3Acceleration = 0;
    }
}
