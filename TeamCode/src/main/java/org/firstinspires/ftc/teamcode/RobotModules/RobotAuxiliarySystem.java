package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;
import org.firstinspires.ftc.teamcode.Sensors.TOFDistanceSensor;

import java.util.HashMap;

public class RobotAuxiliarySystem extends RobotModule {
    private HardwareMap hardwareMap;
    private ColorDistanceSensor colorDistanceSensor;
    private TOFDistanceSensor tofDistanceSensor;

    /**
     * construction method of robot auxiliary system
     */
    public RobotAuxiliarySystem() {
        super("Robot-Auxiliary-System");
    }

    /**
     * initialize the encoders
     *
     * @param dependentModules: not needed
     * @param dependentInstances
     *                          HardwareMap "hardwareMap": connection to the robot's hardware
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
        if (!dependentInstances.containsKey("hardwareMap")) throw new NullPointerException(
                "dependency <<" + "hardwareMap" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentInstances.containsKey("colorDistanceSensor")) throw new NullPointerException(
                "dependency <<" + "colorDistanceSensor" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentInstances.containsKey("tofDistanceSensor")) throw new NullPointerException(
                "dependency <<" + "tofDistanceSensor" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );

        /* get the given instances */
        hardwareMap = (HardwareMap) dependentInstances.get("hardwareMap");
        colorDistanceSensor = (ColorDistanceSensor) dependentInstances.get("colorDistanceSensor");
        tofDistanceSensor = (TOFDistanceSensor) dependentInstances.get("tofDistanceSensor");

    }

    /** not supported yet */
    @Override @Deprecated
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    @Override
    public void periodic() {
        
    }
}
