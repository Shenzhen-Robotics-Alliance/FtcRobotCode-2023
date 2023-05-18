package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drivers.ChassisDriver;
import org.firstinspires.ftc.teamcode.Roboseed_DualPilot;
import org.firstinspires.ftc.teamcode.Roboseed_Test;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;

import java.util.HashMap;
// TODO: add explanation
public class RobotAuxiliarySystem extends RobotModule {
    /** the range at which the robot looks for the cone */
    private static final double aimRange = Math.toRadians(120);
    /** the rotational speed, in motor speed, of the aim */
    private static final double aimSpeed = 0.5;
    /** the rotation tolerance when trying to face the sleeve */
    private static final double rotationTolerance = Math.toRadians(5);

    /** the range to look for the high tower, in cm */
    private static final double highTowerSearchRange = 80;
    /** the range to look for the mid tower, in cm */
    private static final double midTowerSearchRange = 110; // the arm is farther away when reaching for middle
    /** the range to look for the low tower, in cm */
    private static final double lowTowerSearchRange = 40;

    /** the best dropping spot for the high tower, in cm */
    private static final double highTowerDroppingSpot = 50;
    /** the best dropping spot for the mid tower, in cm */
    private static final double midTowerDroppingSpot = 45; // the arm is farther away when reaching for middle
    /** the best dropping spot for the low tower, in cm */
    private static final double lowTowerDroppingSpot = 00; // TODO find this value

    private Arm arm;
    private ChassisDriver chassisDriver;
    private ColorDistanceSensor colorDistanceSensor;
    private DistanceSensor tofDistanceSensor;
    private RobotPositionCalculator positionCalculator;
    /** the creator of the aux */
    private OpMode creator;

    /**
     * the status code of the robot
     *
     * (for sleeves aiming)
     * -1: the system is not started yet
     * 0: the robot auxiliary system is disabled as the pilot didn't ask it to turn on yet or the pilot interrupted it
     * 1: no targets found in the middle, the robot should spin left to find it; if found target, go to 3; if not, go to 2;
     * 2: after no targets found on the left, the robot should seek it on the right; if found, go to 3; if not, stop the process and go to -1;
     * 3: found a target, the robot should turn to face the target
     * 4: the robot's claw is lined up with the target, the robot should move forward until the target lands inside the intake spot; then, when the target lands in the intake spot, it closes the claw
     * IMPORTANT: the robot can only be interrupted at stage 1, 2 and 3; at stage 4, pushing the aim bottom is required to stop the process
     *
     * (for tower aiming)
     * (-1, 0, 1, 2, 3: same as above)
     * 4: the robot will raise its arm and go to the target, and drop the sleeve, the robot goes for the high tower by default, the pilot can select which tower by pressing a, b, x and aim buttom at the same time
     *
     * */
    // private short statusCode = -1;
    private short statusCode = -1;
    /** the code of the target, 0 for sleeves and 1 for towers */
    private short targetCode = 0;

    /** an independent thread that reads the tof distance sensor, as it takes a bit long, 30ms */
    private Thread tofSensorReadingThread;
    /** whether the tof distance sensor reading thread is running */
    private boolean tofDistanceSensorReadingThreadActivated = false;

    /** the robot's rotation the moment the pilot sends the start-aiming command */
    private double startingRotation;
    /** minimum distance location */
    private double minDistanceSpot;
    /** minimum distance to target */
    private double minDistance;
    /** whether any target locked in this scan */
    private boolean targetFound;
    /** the reading of the tof distance sensor */
    private double tofDistanceSensorReading;

    /**
     * construction method of robot auxiliary system
     */
    public RobotAuxiliarySystem() {
        super("Robot-Auxiliary-System");
    }

    /** @Deprecated: this is not going to work as not opmode is running it */
    @Override @Deprecated
    public void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) throws NullPointerException {
        init(dependentModules, dependentInstances, new Roboseed_DualPilot());
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
     * @param creator the op-mode that created this module, so that the thread can be stopped when the program ends
     */
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances,
            LinearOpMode creator) throws NullPointerException
    {
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
        tofDistanceSensor = (DistanceSensor) dependentInstances.get("tofDistanceSensor");
        positionCalculator = (RobotPositionCalculator) dependentModules.get("positionCalculator");

        statusCode = 0;

        this.tofSensorReadingThread = new Thread(new Runnable() {
            @Override public void run() {
                tofDistanceSensorReadingThreadActivated = true;
                while (creator.opModeIsActive() && !creator.isStopRequested()) tofDistanceSensorReading = tofDistanceSensor.getDistance(DistanceUnit.CM);
            }
        });
    }

    /** not supported yet */
    @Override @Deprecated
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {}

    @Override
    public void periodic() {
        System.out.println(statusCode + "," + targetCode);
        if (!tofDistanceSensorReadingThreadActivated) tofSensorReadingThread.start();
        if (targetCode == 0) aimCone();
        else aimTower();
    }

    private void aimCone() {
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
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) < 0) break; // go to the next loop if not reached yet
                chassisDriver.setRotationalMotion(0);
                if (targetFound) statusCode = 3;
                else statusCode = 0;
                chassisDriver.aimStopped();
                break;
            }

            case 3: {
                chassisDriver.setTargetedRotation(minDistanceSpot);
                double rotationalError = ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), minDistanceSpot);
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

    private void aimTower() {
        switch (statusCode) {
            case 1: {
                chassisDriver.setRotationalMotion(-aimSpeed);
                double targetedDirection = startingRotation + (aimRange/2);
                if (tofDistanceSensor.getDistance(DistanceUnit.CM) < 30) {
                    minDistanceSpot = positionCalculator.getRobotRotation();
                    statusCode = 3;
                }
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) > 0) break; // go to the next loop
                chassisDriver.setRotationalMotion(0);
                statusCode = 2;
                break;
            }
            case 2: {
                chassisDriver.setRotationalMotion(aimSpeed);
                double targetedDirection = startingRotation - (aimRange/2);
                if (tofDistanceSensor.getDistance(DistanceUnit.CM) < 30) {
                    minDistanceSpot = positionCalculator.getRobotRotation();
                    statusCode = 3;
                }
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) < 0) break; // go to the next loop
                /* end the aiming process */
                chassisDriver.setRotationalMotion(0);
                statusCode = 0;
                break;
            }
            case 3: {
                chassisDriver.setTargetedRotation(minDistanceSpot);
                double rotationalError = ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), minDistanceSpot);
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
                
            }
        }
    }

    /** call the system to start the aiming process */
    public void startAim() {
        if (statusCode == -1) return;
        // stopAim();
        startingRotation = positionCalculator.getRobotRotation();
        if (arm.getClaw()) targetCode = 1; // if the claw is closed, look for the tower
        else {
            targetCode = 0; // the default target is sleeve
            if (colorDistanceSensor.targetInRange()) { // if the target is already ahead
                statusCode = 4;
                return;
            }
        }
        statusCode = 1;
        minDistance = 1;
        targetFound = false;
        chassisDriver.newAimStarted();
    }

    /** cancel the aiming process */
    public void stopAim() {
        if (statusCode != -1) statusCode = 0;
    }
}