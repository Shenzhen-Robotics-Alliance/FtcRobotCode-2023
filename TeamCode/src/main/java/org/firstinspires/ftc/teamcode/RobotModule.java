package org.firstinspires.ftc.teamcode;

/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: RobotModule.java
 *
 * the class that contains some basic functions for a robot module
 * TODO write this method and imply it in all the modules
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */

import java.util.Map;

public interface RobotModule {
    /*
     * the method called when initializing the robot
     * this is a default method, overwrite it!
     *
     * @param dependentModules: all the modules needed for the this module
     * @return Nah
     * @throws Nah
     * */
    default void init(Map<String, RobotModule> dependentModules) {}

    /*
     * the method called in every loop
     * this is a default method, overwrite it!
     *
     * @param dependentModules: all the modules needed for the this module
     * @return Nah
     * @throws Nah
     * */
    default void periodic() {}
}

