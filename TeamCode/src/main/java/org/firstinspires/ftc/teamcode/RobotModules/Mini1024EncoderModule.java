package org.firstinspires.ftc.teamcode.RobotModules;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotModule;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: Mini1024EncoderModule
 *
 * the module that reads the angular position, velocity and acceleration of the encoders
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
*/
public class Mini1024EncoderModule implements RobotModule {
    /* the current position of the encoders, updated every period of the run loop */
    private double encoder1_position, encoder2_position, encoder3_position;
    /* the current velocity of the encoders, updated every period of the run loop, determined using the change in position and the difference in time */
    private double encoder1_velocity, encoder2_velocity, encoder3_velocity;
    /* the current position of the encoders, updated every period of the run loop, determined using the change in velocity and the difference in time */
    private double encoder1_acceleration, encoder2_acceleration, encoder3_acceleration;

    /**
     * initialize the encoders
     *
     * @param dependentModules: not needed
     * @param dependentInstances:
     *                          DcMotorEx "encoder-1-instance", "encoder-2-instance", "encoder-3-instance3":
     *                          the instance of the three encoders
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) {
        /* get the three encoders from the args */
        DcMotorEx encoder1, encoder2, encoder3;
        encoder1 = (DcMotorEx) dependentInstances.get("encoder-1-instance");
        encoder2 = (DcMotorEx) dependentInstances.get("encoder-2-instance");
        encoder3 = (DcMotorEx) dependentInstances.get("encoder-3-instance");
    }

    /**
     * called every period of run loop
     * get the position, and use it to calculate the velocity and acceleration
     */
    @Override
    public void periodic() {

    }

    /**
     * get the positon of an encoder
     *
     * @param id: the id of the encoder, 1 2 or 3
     * @return position: the current position of the selected encoder
     */

}
