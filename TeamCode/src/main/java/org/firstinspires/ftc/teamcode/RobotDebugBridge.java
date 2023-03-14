package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: DebugBridge.java
 *
 * print the messages that the robot modules want to display
 * TODO write this method
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.13
 * @Version v0.0.2
 */
public class RobotDebugBridge {
    /** the telemetry console that connects to the driver station panel */
    private Telemetry driverStationDebugPort;

    /** the stream that connects to the output terminal on the computer, default System.out */
    private PrintStream consoleDebugPort = System.out;

    /** the array list that contains all the robot's modules */
    private List<RobotModule> robotModules = new ArrayList<>();

    /**
     * construction function of RobotDebug Bridge
     * set the telemetry port and the debug console stream
     *
     * @param driverStationDebugPort: the telemetry port that connects to the driver station, gained in the linear op-mode class
     * @param consoleDebugPort: the stream that connects to the computer's console, for example: System.out
     * @param robotModules: an array list that contains all the modules whose debugging messages need to be displayed
     */
    public RobotDebugBridge(Telemetry driverStationDebugPort, PrintStream consoleDebugPort, ArrayList<RobotModule> robotModules) {
        this.driverStationDebugPort = driverStationDebugPort;
        this.consoleDebugPort = consoleDebugPort;
        this.robotModules = robotModules;
    }

    /**
     * construction function of RobotDebug Bridge, using default console stream
     * set the telemetry port, leave the debug console stream to be default(System.out)
     *
     * @param driverStationDebugPort: the telemetry port that connects to the driver station, gained in the linear op-mode class
     * @param robotModules: an array list that contains all the modules whose debugging messages need to be displayed
     */
    public RobotDebugBridge(Telemetry driverStationDebugPort, ArrayList<RobotModule> robotModules) {
        this.driverStationDebugPort = driverStationDebugPort;
        this.robotModules = robotModules;
    }

    /**
     * set the console messages of a selected module to be visible or not
     *
     * @param module the selected module
     * @param visibility set the console messages of the module to be visible or not
     */
    public void setRobotModuleConsoleMessageVisibility(RobotModule module, boolean visibility) {
        for (RobotModule i: robotModules) {
            // TODO find the targeted module and change it's visibility
            // TODO add a variable(haven't decided where to defined though) that represents the visibility of all the modules
        }
    }

    /**
     * update the debugging messages of all the robot modules
     * meanwhile, show which module displayed each message and the time point it's updated
     * as well as the timestamp between the last update
     * @TODO write this method and display the messages
     */
    public void updateDebuggingMessages() {
        /* go through all the  */
    }
}
