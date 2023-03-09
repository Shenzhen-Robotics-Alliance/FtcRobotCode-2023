package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: RobotModule.java
 *
 * the class that contains some basic functions for a robot module
 * TODO write this method and imply it in all the modules
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.9
 * @Version v0.1.0
*/
public interface RobotModule {
    /**
     * the name of the module
     */
    String moduleName = null;


    /**
     * the method called when initializing the robot
     * this is a default method, overwrite it!
     *
     * @param dependentModules: all the modules needed for this module
     * @param dependentInstances: the instance needed for this module
     */
    default void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) {}

    /**
     * the method called in every loop
     * this is a default method, overwrite it!
     */
    default void periodic() {}

    /**
     * the messages that the module wants to print in the present period
     * this list will be cleared when it is printed out
     */
    List<String> consoleMessages = new ArrayList<>();

    /**
     * get the messages that the module needs to print on the console
     * so the main program can add "[RobotModules/@moduleName]" before each line, for easier debugging
     * TODO print these messages in the console every period (do it in the main program)
     *
     * @return the messages that this module want to print
     */
    default List<String> getDebugConsoleMessages() {
        /* process the message */
        List<String> processedConsoleMessages = new ArrayList<>();
        for(String message: consoleMessages) {
            processedConsoleMessages.add("[RobotModules/" + moduleName + "]" + message);
        }
        return processedConsoleMessages;
    }

    /**
     * the keys for the telemetry
     */
    List<String> telemetryMessagesKeys = new ArrayList<>();
    // TODO write a map to store all the telemetry messages' data

    /**
     * get the messages that the module needs to print on the console
     * so the main program can add "[RobotModules/@moduleName]" before each line, for easier debugging
     *
     * @return the messages that this module want to print
     */
    default List<String> getTelemetryMessages() {
        return consoleMessages;
    }
}

