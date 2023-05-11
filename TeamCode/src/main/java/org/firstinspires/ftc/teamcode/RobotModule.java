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
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.9
 * @Version v0.0.3
*/
public abstract class RobotModule {
    /**
     * the name of the module
     * this variable is used to identify the module, it must be unique
     * override this variable
     */
    private final String moduleName;

    /**
     * construct method of robot module
     * sets the name of the module
     *
     * @param moduleName the name of the module, must be unique as it is used as an identifier
     */
    public RobotModule(String moduleName) {
        this.moduleName = moduleName;
    }

    /**
     * the method called when initializing the robot
     * this is an abstract method, overwrite it!
     *
     * @param dependentModules all the modules needed for this module
     * @param dependentInstances the instance needed for this module
     * @throws NullPointerException if a none-exist module or instance key is given in the hashmaps
     */
    public abstract void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) throws NullPointerException;

    /**
     * update the an instance that is needed in the module to a newer one
     *
     * @param instanceName the name of instance that needs to be replaced
     * @param newerInstance the newer instance that
     *
     * @throws NullPointerException when attempting to update a none-exist instance
     */
    public abstract void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException;

    /**
     * the method called in every loop
     * this is an abstract method, overwrite it with the code of each module!
     */
    public abstract void periodic();

    /**
     * TODO add an abstract function for all the modules that's called at the end of the program, to destroy some instances, free-up some memories and disable motors
     */
     // public abstract void terminate();

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
    public List<String> getDebugConsoleMessages() {
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
    Map<String, RobotModule> telemetryMessages = new HashMap<String, RobotModule>();

    /**
     * get the messages that the module needs to print on the console
     * so the main program can add "[RobotModules/@moduleName]" before each line, for easier debugging
     *
     * @return the messages that this module want to print
     */
    public List<String> getTelemetryMessages() {
        return consoleMessages;
    }

    /**
     * returns the name of the module
     *
     * @return the name of the module, in String, used to identify the module
     */
    public String getModuleName() {
        return this.moduleName;
    }

    /**
     * magic method to that turns the object to string
     * called when a robot module is turned to string
     *
     * @return a message including the name and running status
     */
    @Override
    public String toString() {
        return "<--Robot module, name: " + getModuleName() + ", status: running without error-->";
    }
}

