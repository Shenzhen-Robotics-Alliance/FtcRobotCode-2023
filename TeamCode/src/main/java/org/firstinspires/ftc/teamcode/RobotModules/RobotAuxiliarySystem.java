package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drivers.ChassisDriver;
import org.firstinspires.ftc.teamcode.Roboseed_DualPilot;
import org.firstinspires.ftc.teamcode.RobotModule;
import org.firstinspires.ftc.teamcode.Sensors.ColorDistanceSensor;

import java.util.HashMap;

// TODO: add explanation
public class RobotAuxiliarySystem extends RobotModule {
    /** the range at which the robot looks for the cone */
    private static final double aimRange = Math.toRadians(120);
    /** the rotational speed, in motor speed, of the aim */
    private static final double aimSpeed = 0.35;
    /** the rotation tolerance when trying to face the sleeve */
    private static final double rotationTolerance = Math.toRadians(3);

    /** the tolerance for translational error, in encoder values */
    private static final double encoderErrorTolerance = 250;

    /** the range to look for the high tower, in cm */
    private static final double highTowerSearchRange = 65;
    /** the range to look for the mid tower, in cm */
    private static final double midTowerSearchRange = 110; // the arm is farther away when reaching for middle
    /** the range to look for the low tower, in cm */
    private static final double lowTowerSearchRange = 40;
    private static final double[] searchRangeList = {0, lowTowerSearchRange, midTowerSearchRange, highTowerSearchRange};

    /** the best dropping spot for the high tower, in cm */
    private static final double highTowerDroppingSpot = 42;
    /** the best dropping spot for the mid tower, in cm */
    private static final double midTowerDroppingSpot = 45; // the arm is farther away when reaching for middle
    /** the best dropping spot for the low tower, in cm */
    private static final double lowTowerDroppingSpot = 00; // TODO find this value
    private static final double[] droppingSpotList = {0, lowTowerDroppingSpot, midTowerDroppingSpot, highTowerDroppingSpot};
    private static final double[] droppingSpotListEdge = {0, lowTowerDroppingSpot, midTowerDroppingSpot, 52};

    /** the angle between the sensor's aim center and the center of the arm, when approaching it from the left side */
    private static final double aimCenterToDropCenterAngleLeft = Math.toRadians(0);
    /** when approaching as rotating to the right */
    private static final double aimCenterToDropCenterAngleRight = Math.toRadians(-4);
    /** when doing high-speed aim, which is to say, measure the rotation of one edge of the tower */
    private static final double aimEdgeToDropCenterAngleLeft = Math.toRadians(8);
    /** when approaching as rotating to the right at high speed*/
    private static final double aimEdgeToDropCenterAngleRight = Math.toRadians(6);

    private static final double encoderValuePerCM = 6450 / 30; // measured that 6000 encoder values where increased for a 30cm of move

    private Arm arm;
    private ChassisDriver chassisDriver;
    private ColorDistanceSensor colorDistanceSensor;
    private DistanceSensor tofDistanceSensor;
    private RobotPositionCalculator positionCalculator;

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
     * 4: the robot will raise its arm and go to the target, and lower it's arm to score sleeves and open claw once the towers holds the sleeves,
     * IMPORTANT: the robot goes for the high tower by default, the pilot can select which tower by pressing a, b, x and aim bottom at the same time
     * */
    // private short statusCode = -1;
    public int statusCode = -1;
    /** the code of the target, 0 for sleeves and 1 for towers */
    private int targetCode = 0;
    private boolean highSpeedAim;

    /** an independent thread that reads the tof distance sensor, as it takes a bit long, 30ms */
    private Thread tofSensorReadingThread;
    /** whether the tof distance sensor reading thread is running */
    private boolean tofDistanceSensorReadingThreadActivated = false;

    /** the robot's rotation the moment the pilot sends the start-aiming command */
    private double startingRotation;
    /** the rotation at which minimum distance to sleeves occures */
    private double minDistanceSpot;
    /** minimum distance to the sleeve */
    private double minDistance;
    /** whether any target locked in this scan */
    private boolean targetFound;

    /** the distance to tower the first time it was found */
    private double towerDistance;
    /** the rotational of the robot when the tower is seen by the sensors */
    private double towerRotation;
    /** the spot of the tower, calculated into absolute encoder position */
    private double[] towerPosition = new double[2];
    /** the reading of the tof distance sensor */
    public double tofDistanceSensorReading;

    /**
     * construction method of robot auxiliary system
     */
    public RobotAuxiliarySystem() {
        super("Robot-Auxiliary-System");
    }

    /** @Deprecated: this is not going to work as not opmode is running it */
    @Override @Deprecated
    public void init(HashMap<String, RobotModule> dependentModules, HashMap<String, Object> dependentInstances) throws NullPointerException {
        init(dependentModules, dependentInstances, new Roboseed_DualPilot(), false);
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
     * @param enableHighSpeedAiming whether to enable fast aiming, or do precise but slow aim
     */
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances,
            LinearOpMode creator,
            boolean enableHighSpeedAiming) throws NullPointerException
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
        this.highSpeedAim = enableHighSpeedAiming;
        // this.chassisDriver.setFastModeOn(enableHighSpeedAiming);
        this.chassisDriver.setFastModeOn(false); // TODO get this done

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
        if (!tofDistanceSensorReadingThreadActivated) tofSensorReadingThread.start();

        if (chassisDriver.isAimProcessInterrupted()) targetCode = 0;

        if (statusCode == 0) chassisDriver.aimStopped();

        if (targetCode == 0) aimCone();
        else if (highSpeedAim) aimTowerFast();
        else aimTower();
    }

    private void aimCone() {
        switch (statusCode) {
            case 1: {
                chassisDriver.setRotationalMotion(-aimSpeed);
                double targetedDirection = startingRotation + (aimRange /2);
                /* if the color distance captured anything */
                if (colorDistanceSensor.targetInRange()) {
                    targetFound = true;
                    minDistance = Math.min(colorDistanceSensor.getDistanceToTarget(), minDistance);
                    minDistanceSpot = positionCalculator.getRobotRotation();
                }
                /* wait for the robot to turn left, until difference is negative */
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) > 0 && !targetFound) break; // go to the next loop
                chassisDriver.setRotationalMotion(0);
                if (targetFound) {
                    statusCode = 3;
                    chassisDriver.setTargetedRotation(minDistanceSpot);
                }
                else statusCode = 2;
                break;
            }

            case 2: {
                chassisDriver.setRotationalMotion(aimSpeed);
                double targetedDirection = startingRotation - (aimRange /2);
                /* if the color distance captured anything */
                if (colorDistanceSensor.targetInRange()) {
                    targetFound = true;
                    minDistance = Math.min(colorDistanceSensor.getDistanceToTarget(), minDistance);
                    minDistanceSpot = positionCalculator.getRobotRotation();
                }
                /* wait for the robot to turn right, until difference is positive */
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) < 0 && !targetFound) break; // go to the next loop if not reached yet
                chassisDriver.setRotationalMotion(0);
                if (targetFound) {
                    statusCode = 3;
                    chassisDriver.setTargetedRotation(minDistanceSpot);
                }
                else statusCode = 0;
                chassisDriver.aimStopped();
                break;
            }

            case 3: {
                double rotationalError = ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), minDistanceSpot);
                /* positionCalculator.forceUpdateEncoderValue(); positionCalculator.periodic(); // if used in auto stage */
                chassisDriver.sendCommandsToMotors();
                if (Math.abs(rotationalError) < rotationTolerance) {
                    chassisDriver.switchToManualRotationMode();
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
        System.out.println(statusCode);
        switch (statusCode) {
            case 1: {
                chassisDriver.setRotationalMotion(-aimSpeed);
                double targetedDirection = startingRotation + (aimRange /2);
                /* if target is ahead */
                if (tofDistanceSensorReading < searchRangeList[targetCode]) {
                    System.out.println("target in range and is found:" + targetFound + "; rotation" + Math.toDegrees(positionCalculator.getRobotRotation()));
                    /* if this is not the first time to see the target */
                    if (targetFound) break; // wait for the target to disappear
                    /* record the information's for further calculation */
                    towerRotation = positionCalculator.getRobotRotation();
                    towerDistance = tofDistanceSensorReading;
                    targetFound = true;
                } else if (targetFound) { // when the target is lost again
                    statusCode = 3; // go for it
                    towerRotation = ChassisDriver.midPoint(towerRotation, positionCalculator.getRobotRotation()); // set it's rotation to be mid point between it's two edges
                    towerRotation += aimCenterToDropCenterAngleLeft;
                    System.out.println("tower found:"+ Math.toDegrees(towerRotation));
                }
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) > 0 || !targetFound) break; // go to the next loop
                chassisDriver.setRotationalMotion(0);
                statusCode = 3;
                break;
            }
            case 2: {
                chassisDriver.setRotationalMotion(aimSpeed);
                double targetedDirection = startingRotation - (aimRange /2);
                if (tofDistanceSensorReading < searchRangeList[targetCode]) {
                    System.out.println("target in range and is found:" + targetFound + "; rotation" + Math.toDegrees(positionCalculator.getRobotRotation()));
                    if (targetFound) break;
                    towerRotation = positionCalculator.getRobotRotation();
                    towerDistance = tofDistanceSensorReading;
                    targetFound = true;
                } else if (targetFound) {
                    statusCode = 3;
                    towerRotation = ChassisDriver.midPoint(towerRotation, positionCalculator.getRobotRotation());
                    towerRotation += aimCenterToDropCenterAngleRight;
                    System.out.println("tower found:"+ Math.toDegrees(towerRotation));
                }
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) < 0 || !targetFound) break; // go to the next loop
                /* end the aiming process */
                chassisDriver.setRotationalMotion(0);
                statusCode = 3;
                break;
            }
            case 3: {
                chassisDriver.setTargetedRotation(towerRotation);
                double rotationalError = ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), towerRotation);
                /* positionCalculator.forceUpdateEncoderValue(); positionCalculator.periodic(); // if used in auto stage */
                chassisDriver.sendCommandsToMotors();
                System.out.println("rotational error:" + rotationalError);
                if (Math.abs(rotationalError) < rotationTolerance) {
                    chassisDriver.switchToManualRotationMode();
                    chassisDriver.setRotationalMotion(0);

                    double distanceToDroppingSpot = (towerDistance - droppingSpotList[targetCode]) * encoderValuePerCM;
                    towerPosition[0] = positionCalculator.getRobotPosition()[0] + Math.cos(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot;
                    towerPosition[1] = positionCalculator.getRobotPosition()[1] + Math.sin(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot;
                    System.out.println(Math.sin(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot + "," + Math.cos(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot);
                    chassisDriver.setTargetedTranslation_fixedRotation(towerPosition[0], towerPosition[1]);
                    statusCode = 4;
                }
                break;
            }
            case 4: {
                // TODO if the robot is not moving at all, stop the dead loop; adjust the PIDs, the robot is stuck when being told to move to a nearby position

                chassisDriver.sendCommandsToMotors();
                double xAxisDifference = positionCalculator.getRobotPosition()[0] - towerPosition[0];
                double yAxisDifference = positionCalculator.getRobotPosition()[1] - towerPosition[1];
                if (xAxisDifference * xAxisDifference + yAxisDifference * yAxisDifference > encoderErrorTolerance * encoderErrorTolerance) break; // keep waiting
                /* if the robot reached the tower */
                chassisDriver.setRotationalMotion(0);
                chassisDriver.setRobotTranslationalMotion(0, 0);
                chassisDriver.sendCommandsToMotors();
                arm.lowerArm();
                ElapsedTime descendTime = new ElapsedTime();
                while (descendTime.milliseconds() < 300) {
                    arm.periodic();
                }
                arm.openClaw();
                statusCode = 0;
                break;
            }
        }
    }

    private void aimTowerFast() {
        System.out.println(statusCode);
        switch (statusCode) {
            case 1: {
                chassisDriver.setRotationalMotion(-aimSpeed * 1.6);
                double targetedDirection = startingRotation + (aimRange /2);
                /* if target is ahead */
                if (tofDistanceSensorReading < searchRangeList[targetCode]) {
                    System.out.println("target in range and is found:" + targetFound + "; rotation" + Math.toDegrees(positionCalculator.getRobotRotation()));
                    towerRotation = positionCalculator.getRobotRotation();
                    towerRotation += aimEdgeToDropCenterAngleLeft;
                    towerDistance = tofDistanceSensorReading;
                    targetFound = true;
                    statusCode = 3; // go for it
                }
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) > 0) break; // go to the next loop
                chassisDriver.setRotationalMotion(0);
                statusCode = 0;
                break;
            }
            case 2: {
                chassisDriver.setRotationalMotion(aimSpeed * 1.6);
                double targetedDirection = startingRotation - (aimRange /2);
                if (tofDistanceSensorReading < searchRangeList[targetCode]) {
                    towerRotation = positionCalculator.getRobotRotation();
                    towerRotation += aimEdgeToDropCenterAngleRight;
                    towerDistance = tofDistanceSensorReading;
                    targetFound = true;
                    statusCode = 3; // go for it
                }
                if (ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), targetedDirection) < 0) break; // go to the next loop
                /* end the aiming process */
                chassisDriver.setRotationalMotion(0);
                statusCode = 0;
                break;
            }
            case 3: {
                chassisDriver.setTargetedRotation(towerRotation);
                double rotationalError = ChassisDriver.getActualDifference(positionCalculator.getRobotRotation(), towerRotation);
                /* positionCalculator.forceUpdateEncoderValue(); positionCalculator.periodic(); // if used in auto stage */
                chassisDriver.sendCommandsToMotors();
                System.out.println("rotational error:" + rotationalError);
                if (Math.abs(rotationalError) < rotationTolerance) {
                    chassisDriver.switchToManualRotationMode();
                    chassisDriver.setRotationalMotion(0);

                    double distanceToDroppingSpot = (towerDistance - droppingSpotListEdge[targetCode]) * encoderValuePerCM;
                    towerPosition[0] = positionCalculator.getRobotPosition()[0] + Math.cos(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot;
                    towerPosition[1] = positionCalculator.getRobotPosition()[1] + Math.sin(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot;
                    System.out.println(Math.sin(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot + "," + Math.cos(positionCalculator.getRobotRotation() + Math.toRadians(90)) * distanceToDroppingSpot);
                    chassisDriver.setTargetedTranslation_fixedRotation(towerPosition[0], towerPosition[1]);
                    statusCode = 4;
                }
                break;
            }
            case 4: {
                // TODO if the robot is not moving at all, stop the dead loop; adjust the PIDs, the robot is stuck when being told to move to a nearby position

                chassisDriver.sendCommandsToMotors();
                double xAxisDifference = positionCalculator.getRobotPosition()[0] - towerPosition[0];
                double yAxisDifference = positionCalculator.getRobotPosition()[1] - towerPosition[1];
                if (xAxisDifference * xAxisDifference + yAxisDifference * yAxisDifference > encoderErrorTolerance * encoderErrorTolerance) break; // keep waiting
                /* if the robot reached the tower */
                chassisDriver.setRobotTranslationalMotion(0, 0);
                chassisDriver.sendCommandsToMotors();
                arm.lowerArm();
                ElapsedTime descendTime = new ElapsedTime();
                while (descendTime.milliseconds() < 300) {
                    arm.periodic();
                    chassisDriver.setRotationalMotion(0);
                }
                arm.openClaw();
                statusCode = 0;
                break;
            }
        }
    }

    /**
     * call the system to start the aiming process
     * @param targetCode the code of the target, 0 for sleeves, 1 2 3 for low to high towers
     * @param targetDirection the direction to search the target, 1 for left and 2 for right
     * */
    public void startAim(int targetCode, int targetDirection) {
        if (statusCode == -1) return;
        // stopAim();
        startingRotation = positionCalculator.getRobotRotation();
        this.targetCode = targetCode;
        if (targetCode == 0) {
            if (colorDistanceSensor.targetInRange()) { // if the target is already ahead
                statusCode = 4;
                return;
            }
        }
        statusCode = targetDirection;
        minDistance = 1;
        targetFound = false;
        chassisDriver.newAimStarted();
    }

    /**
     * calls the system to start aim
     * @param targetDirection the direction to search the target, 1 for left and 2 for right
     * */
    public void startAim(int targetDirection) {
        int targetCode;
        if (arm.getClaw()) targetCode = 3; // go for high tower by default
        else targetCode = 0;
        startAim(targetCode, targetDirection);
    }

    public void startAim() {
        startAim(0, 0);
    }

    /** cancel the aiming process */
    public void stopAim() {
        if (statusCode != -1) statusCode = 0;
    }
}
