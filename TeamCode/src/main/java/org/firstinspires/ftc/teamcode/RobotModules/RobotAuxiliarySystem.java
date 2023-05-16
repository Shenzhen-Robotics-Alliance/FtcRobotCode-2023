package org.firstinspires.ftc.teamcode.RobotModules;

import org.firstinspires.ftc.teamcode.Drivers.ChassisDriver;
import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;
import org.firstinspires.ftc.teamcode.Sensors.TOFDistanceSensor;

import java.util.HashMap;
// TODO: add explanation
public class RobotAuxiliarySystem extends RobotModule {
    /** the range at which the robot looks for the cone */
    private static final double aimRange = Math.toRadians(120);
    /** the rotational speed, in motor speed, of the aim */
    private static final double aimSpeed = 0.5;
    /** the rotation tolerance when trying to face the sleeve */
    private static final double rotationTolerance = Math.toRadians(5);

    private Arm arm;
    private ChassisDriver chassisDriver;
    private ColorDistanceSensor colorDistanceSensor;
    private TOFDistanceSensor tofDistanceSensor;
    private RobotPositionCalculator positionCalculator;

    /**
     * the status code of the robot
     * -1: the system is not started yet
     * 0: the robot auxiliary system is disabled as the pilot didn't ask it to turn on yet or the pilot interrupted it
     * 1: no targets found in the middle, the robot should spin left to find it; if found target, go to 3; if not, go to 2;
     * 2: after no targets found on the left, the robot should seek it on the right; if found, go to 3; if not, stop the process and go to -1;
     * 3: found a target, the robot should turn to face the target
     * 4: the robot's claw is lined up with the target, the robot should move forward until the target lands inside the intake spot; then, when the target lands in the intake spot, it closes the claw
     * IMPORTANT: the robot can only be interrupted at stage 1, 2 and 3; at stage 4, pushing the aim bottom is required to stop the process
     * */
    // private short statusCode = -1;
    public short statusCode = -1;
    /** the robot's rotation the moment the pilot sends the start-aiming command */
    private double startingRotation;
    /** minimum distance location */
    private double minDistanceSpot;
    /** minimum distance to target */
    private double minDistance;
    /** whether any target locked in this scan */
    private boolean targetFound;

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
     *                          Arm "arm": connection to the robot's arm and claw
     * @param dependentInstances
     *                          ChassisDriver "chassisDriver": connection to the robot's chassis
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
        if (!dependentModules.containsKey("arm")) throw new NullPointerException(
                "dependency <<" + "arn" + ">> not specified for module <<" + this.getModuleName() + ">>"
        );
        if (!dependentInstances.containsKey("chassisDriver")) throw new NullPointerException(
                "dependency <<" + "chassisDriver" + ">> not specified for module <<" + this.getModuleName() + ">>"
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
        arm = (Arm) dependentModules.get("arm");
        chassisDriver = (ChassisDriver) dependentInstances.get("chassisDriver");
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
                chassisDriver.setRotationalMotion(-aimSpeed);
                double targetedDirection = startingRotation + (aimRange/2);
                /* if the color distance captured anything */
                if (colorDistanceSensor.targetInRange()) {
                    targetFound = true;
                    minDistance = Math.min(colorDistanceSensor.getDistanceToTarget(), minDistance);
                    minDistanceSpot = positionCalculator.getRobotRotation();
                }
                /* wait for the robot to turn left, until difference is negative */
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) > 0) break; // go to the next loop
                chassisDriver.setRotationalMotion(0);
                if (targetFound) statusCode = 3;
                else statusCode = 2;
                break;
            }

            case 2: {
                chassisDriver.setRotationalMotion(aimSpeed);
                double targetedDirection = startingRotation - (aimRange/2);
                /* if the color distance captured anything */
                if (colorDistanceSensor.targetInRange()) {
                    targetFound = true;
                    minDistance = Math.min(colorDistanceSensor.getDistanceToTarget(), minDistance);
                    minDistanceSpot = positionCalculator.getRobotRotation();
                }
                /* wait for the robot to turn right, until difference is positive */
                chassisDriver.setRotationalMotion(0);
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) < 0) break; // go to the next loop if not reached yet
                if (targetFound) statusCode = 3;
                else statusCode = 0;
                chassisDriver.aimStopped();
                break;
            }

            case 3: {
                chassisDriver.setTargetedRotation(minDistanceSpot);
                double rotationalError;
                rotationalError = ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), minDistanceSpot);
                /* positionCalculator.forceUpdateEncoderValue(); positionCalculator.periodic(); // if used in auto stage */
                chassisDriver.sendCommandsToMotors();
                if (Math.abs(rotationalError) < rotationTolerance) {
                    chassisDriver.switchToManualMode();
                    chassisDriver.setRotationalMotion(0);
                    statusCode = 4;
                }
                break;
            }

            case 4: {
                chassisDriver.setRobotTranslationalMotion(0, 0.35);
                if (colorDistanceSensor.getDistanceToTarget() <= 0.15) {
                    chassisDriver.setRobotTranslationalMotion(0, 0);
                    arm.closeClaw();
                    statusCode = 0;
                }
                break;
            }
        }
    }

    /** call the system to start the aiming process */
    public void startAim() {
        // stopAim();
        startingRotation = positionCalculator.getRobotRotation();
        if (statusCode != -1) statusCode = 1;
        minDistance = 1;
        targetFound = false;
        chassisDriver.newAimStarted();
    }

    /** cancel the aiming process */
    public void stopAim() {
        if (statusCode != -1) statusCode = 0;
    }
}
