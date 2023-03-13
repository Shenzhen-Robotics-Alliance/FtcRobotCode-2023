package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.PrintStream;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: DebugBridge.java
 *
 * print the messages that the robot modules want to display
 * TODO write this method
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.3.13
 * @Version v0.0.1
 */
public class RobotDebugBridge {
    /** the telemetry console that connects to the driver station panel */
    private Telemetry driverStationDebugPort;

    /** the stream that connects to the output terminal on the computer, default System.out */
    private PrintStream consoleDebugPort = System.out;

    /**
     * construction function of RobotDebug Bridge
     * set the telemetry port and the debug console stream
     *
     * @param driverStationDebugPort: the telemetry port that connects to the driver station, gained in the linear op-mode class
     * @param consoleDebugPort: the stream that connects to the computer's console, for example: System.out
     */
    public RobotDebugBridge(Telemetry driverStationDebugPort, PrintStream consoleDebugPort) {
        this.driverStationDebugPort = driverStationDebugPort;
        this.consoleDebugPort = consoleDebugPort;
    }

    /**
     * construction function of RobotDebug Bridge, using default console stream
     * set the telemetry port, leave the debug console stream to be default(System.out)
     *
     * @param driverStationDebugPort: the telemetry port that connects to the driver station, gained in the linear op-mode class
     */
    public RobotDebugBridge(Telemetry driverStationDebugPort) {
        this.driverStationDebugPort = driverStationDebugPort;
    }

    public void
}
