package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;
import org.firstinspires.ftc.teamcode.Sensors.TOFDistanceSensor;

import java.util.HashMap;

public class RobotAuxiliarySystem extends RobotModule {
    /** the range at which the robot looks for the cone */
    private static final double aimRange = Math.toRadians(60);

    private HardwareDriver hardwareDriver;
    private ColorDistanceSensor colorDistanceSensor;
    private TOFDistanceSensor tofDistanceSensor;
    private RobotPositionCalculator positionCalculator;

    /**
     * the status code of the robot
     * -1: the system is not started yet
     * 0: the robot auxiliary system is disabled as the pilot didn't ask it to turn on yet or the pilot interrupted it
     * 1: no targets found in the middle, the robot should spin left to find it; if found target, go to 3; if not, go to 2;
     * 2: after no targets found on the left, the robot should seek it on the right; if found, go to 3; if not, stop the process and go to -1;
     * 3: found a target, the robot should move forward until the target lands inside the intake spot; then, when the target lands in the intake spot, it closes the claw
     * IMPORTANT: the robot can only be interrupted at stage 1 and 2; at stage 3, pushing the aim bottom is required to stop the process
     * */
    private short statusCode = -1;
    /** the robot's rotation the moment the pilot sends the start-aiming command */
    private double startingRotation;

    /**
     * construction method of robot auxiliary system
     */
    public RobotAuxiliarySystem() {
        super("Robot-Auxiliary-System");
    }

    /**
     * initialize the encoders
     *
     * @param dependentModules
     *                          RobotPositionCalculator "positionCalculator"
     * @param dependentInstances
     *                          HardwareDriver "hardwareDriver": connection to the robot's hardware
     *                          ColorDistanceSensor "colorDistanceSensor": color sensor reader
     *                          TOFDistanceSensor "tofDistanceSensor": time-of-flight inferred distance sensor reader
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        /* throw out exceptions if dependencies not specified */
        if (dependentInstances.isEmpty()) throw new NullPointerException (
                "an empty set of dependent instances given to the module<<" + this.getModuleName() + ">> which requires at least one instance(s) as dependency"
        );
        if (dependentModules.isEmpty()) throw new NullPointerException(
                "an empty set of dependent modules given to the module<<" + this.getModuleName() + ">> which requires at least one module(s) as dependency"
        );
        if (!dependentInstances.containsKey("hardwareDriver")) throw new NullPointerException(
                "dependency <<" + "hardwareDriver" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentInstances.containsKey("colorDistanceSensor")) throw new NullPointerException(
                "dependency <<" + "colorDistanceSensor" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentInstances.containsKey("tofDistanceSensor")) throw new NullPointerException(
                "dependency <<" + "tofDistanceSensor" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentModules.containsKey("positionCalculator")) throw new NullPointerException(
                "dependency <<" + "positionCalculator" + ">> not specified for module <<" + this.getModuleName() + ">>");

        /* get the given instances */
        hardwareDriver = (HardwareDriver) dependentInstances.get("hardwareDriver");
        colorDistanceSensor = (ColorDistanceSensor) dependentInstances.get("colorDistanceSensor");
        tofDistanceSensor = (TOFDistanceSensor) dependentInstances.get("tofDistanceSensor");
        positionCalculator = (RobotPositionCalculator) dependentModules.get("positionCalculator");

        statusCode = 0;
    }

    /** not supported yet */
    @Override @Deprecated
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    @Override
    public void periodic() {
        switch (statusCode) {
            case 1: {
                
                break;
            }
        }
    }

    /** call the system to start the aiming process */
    public void startAim() {
        // stopAim();
        if (statusCode != -1) statusCode = 1;
    }

    /** cancel the aiming process */
    public void stopAim() {
        if (statusCode != -1) statusCode = 0;
    }
}
