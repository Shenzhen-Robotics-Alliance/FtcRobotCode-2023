package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

/**
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: ChassisModule.java
 *
 * the program that controls the moving of the robot in manual stage
 * TODO improve readability by adding explanations ot the code; add implements to the methods from super class RobotModule
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 */
public class RobotChassis extends RobotModule { // controls the moving of the robot
    /** the rotation of the control hub, in reference to the chassis, see https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html */
    final double xRotation = 0;
    final double yRotation = 145.64;
    final double zRotation = 0;

    /**
     * the controlling pad used to control the robot's movements.
     * dpad upper button: switch on/off field-navigation mode,
     * dpad lower button: switch on/off slow motion mode,
     * dpad left button: switch reverse y axle,
     * dpad right button: reset IMU YAW direction,
     * */
    private Gamepad gamepad;
    /** connects to the robot's hardware */
    private HardwareDriver driver;
    /** the robot's imu for navigation */
   // private IMU imu;
    /** the module that calculates the robot's position and velocity */
    private RobotPositionCalculator positionCalculator;

    /** whether the pilot asked for slow motion mode */
    private boolean slowMotionModeRequested;
    /** whether the program suggests slow motion mode */
    private boolean slowMotionModeSuggested;
    /** determines whether the slow motion mode is activated eventually */
    private boolean slowMotionModeActivationSwitch;
    /** whether to correct the robot's motion in reference to the pilot's directions */
    private boolean groundNavigatingModeActivationSwitch;
    /** whether to reverse the y axis of the controller, according to the preference of the pilot */
    private boolean yAxleReversedSwitch;

    /** the times elapsed after the last time these mode buttons are pressed
     * so that it does not shift between the modes inside one single press */
    private final ElapsedTime previousMotionModeButtonActivation = new ElapsedTime();
    private final ElapsedTime previousNavigationModeButtonActivation = new ElapsedTime();
    private final ElapsedTime previousYAxleReverseSwitchActivation = new ElapsedTime();
    private final ElapsedTime lastMovement = new ElapsedTime();
    private final ElapsedTime dt = new ElapsedTime();

    /** configuration of the robot's driving feelings */
    /** the minimum power needed to move the robot */
    private final double minDrivingPower = 0;
    /** the minimum stick input value that the robot will respond to, anything smaller than this, the robot ignores */
    private final double minStickValue = 0.05;
    /** the maximum motor power of the robot when it's moving free */
    private final double maxDrivingPower = 1;
    /** the limit fo the motor power when the robot is carrying a goal */
    private final double maxCarryingPower = 0.6;

    /** calibrate the controller, store the initial state of the controller, in the order of x-axis, y axis and rotational axis */
    private double[] pilotControllerPadZeroPosition = {0, 0, 0};

    /** the rotation that the pilot set it to be */
    private double targetedRotation;
    /** the minimum speed when the encoder starts to correct the motion */
    private static final double useEncoderCorrectionSpeed = 1;

    private static final double rotationTolerance = Math.toRadians(5);
    /** the rotational deviation when the robot starts to decelerate */
    private static final double rotationStartsSlowingDown = Math.toRadians(180);

    /** minimum power during to rotate the robot */
    private static final double minMovingMotorPower = 0.05;
    /** motor speed limit */
    private static final double maxMovingMotorPower = 0.65;

    /** the correction factor when using encoders to correct motor speed */
    private static final double encoderRotationToMotorSpeedFactor = -1;

    /**
     * construct function of the robot chassis, use init() for further initialization
     */
    public RobotChassis() {
        super("RobotChassis");
    }

    /**
     * initialize the chassis of the robot
     *
     * @param dependentModules: null should be given as this module does not depend on any other modules
     * @param dependentInstances: the instance needed by the robot's chassis
     *                          "initialControllerPad": Gamepad, the default game pad used to control the robot's chassis
     *                          "hardwareDriver" : HardwareDriver, the connection to the robot's hardware
     *                          "positionCalculator" : RobotPositionCalculator_tmp, the position calculator of the robot
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        /* throw out an error if the dependentInstances is given an empty map */
        if (dependentInstances.isEmpty()) throw new NullPointerException(
                "an empty map of dependent instances given to this module, which requires at least one instant dependencies"
        );


        /* get the instances from the param */
        if (! dependentInstances.containsKey("initialControllerPad")) throw new NullPointerException(
                "dependent instance not given: " + "initialControllerPad"
        );
        this.gamepad = (Gamepad) dependentInstances.get("initialControllerPad");

        if (! dependentInstances.containsKey("hardwareDriver")) throw new NullPointerException(
                "dependent instance not given: " + "hardwareDriver"
        );
        this.driver = (HardwareDriver) dependentInstances.get("hardwareDriver");
//
//        if (! dependentInstances.containsKey("imu")) throw new NullPointerException(
//                "dependent instance not given: " + "imu"
//        );
//        this.imu = (IMU) dependentInstances.get("imu");


        /* throw out an error if the dependent modules is given an empty map */
        if (dependentModules.isEmpty()) throw new NullPointerException(
                "an empty map of dependent modules given to this module, which requires at least one instant dependencies"
        );
        if (! dependentModules.containsKey("positionCalculator")) throw new NullPointerException(
                "dependent instance not given: " + "positionCalculator"
        );
        this.positionCalculator = (RobotPositionCalculator) dependentModules.get("positionCalculator");


//        /* calibrate the imu */
//        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        imu.resetYaw();

        /* calibrate the center of the controller pad */
        this.pilotControllerPadZeroPosition[0] = gamepad.right_stick_x;
        this.pilotControllerPadZeroPosition[1] = gamepad.right_stick_y;
        this.pilotControllerPadZeroPosition[2] = gamepad.left_stick_x;

        this.slowMotionModeRequested = false;
        this.slowMotionModeSuggested = false;
        this.slowMotionModeActivationSwitch = false;
    }

    @Override
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {

    }

    @Override
    public void periodic() {
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        double facing;
        double velocityYAW;
        double[] correctedMotion;

        /* get the controller pad input, and correct it according to the initial  */
        double yAxleMotion = linearMap(-(gamepad.right_stick_y - this.pilotControllerPadZeroPosition[1])); // the left stick is reversed to match the vehicle
        double xAxleMotion = linearMap(gamepad.right_stick_x - this.pilotControllerPadZeroPosition[0]);
        double rotationalAttempt = linearMap(gamepad.left_stick_x -  this.pilotControllerPadZeroPosition[2]); // the driver's attempt to rotate
        if (slowMotionModeActivationSwitch) rotationalAttempt *= 0.5;
        // targetedRotation -= rotationalAttempt * dt.seconds() * maxAngularVelocity;

        boolean movement = xAxleMotion != 0 | yAxleMotion != 0;
        if (groundNavigatingModeActivationSwitch & movement) { // when the pilot chooses to navigate according to the ground, don't apply when the robot is still
//            // get the rotation and angular velocity of the robot from imu
//            orientation = imu.getRobotYawPitchRollAngles();
//            angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
//
//            // get the facing, and the angular velocity in YAW axle, of the robot
//            facing = orientation.getYaw(AngleUnit.RADIANS);
//            velocityYAW = angularVelocity.zRotationRate;

            /* use robot position calculator instead to calculate the rotation of the robot */
            facing = positionCalculator.getRobotRotation();

            // correct xAxelMotion and yAxelMotion using the IMU
            correctedMotion = navigateGround(xAxleMotion, yAxleMotion, -facing);
            xAxleMotion = correctedMotion[0];
            yAxleMotion = correctedMotion[1];
        } else if (yAxleReversedSwitch) yAxleMotion *= -1;

        if (yAxleMotion != 0 | xAxleMotion != 0 | rotationalAttempt != 0) lastMovement.reset();

        /** correct the motion using encoder readings */
        if (positionCalculator.getRawVelocity()[0] * positionCalculator.getRawVelocity()[1] != 0) {
            /* get the current moving direction according to the encoders, in radian */
            double currentDirection = Math.atan(positionCalculator.getRawVelocity()[1] / positionCalculator.getRawVelocity()[0]);
            /* using the current direction that the encoders gave us, determine the robot's current movement, in motor speed(not in encoder value) */
            double motorSpeed = Math.sqrt(xAxleMotion * xAxleMotion + yAxleMotion * yAxleMotion);
            double[] currentMotorVelocity = {motorSpeed * Math.cos(currentDirection), motorSpeed * Math.sin(currentDirection)};
            /* using the current velocity computed above, correct the targeted velocity */
            if (motorSpeed > useEncoderCorrectionSpeed) {
                xAxleMotion += (xAxleMotion - currentMotorVelocity[0]);
                yAxleMotion += (yAxleMotion - currentMotorVelocity[1]);
            }
        }

//        /** rotate the robot to make it stick to the rotation where it's asked to be */
//        double rotationalDifference = AutoStageRobotChassis_tmp.reformatRotationDifference(targetedRotation - positionCalculator.getRobotRotation());
//        double rotationalMotion = Math.copySign(RobotChassis.linearMap(
//                rotationTolerance,rotationStartsSlowingDown,0,0.8,
//                Math.abs(rotationalDifference)
//        ) , rotationalDifference);
        double rotationalMotion = getRotationMotorSpeed(rotationalAttempt);

        // control the Mecanum wheel
        driver.leftFront.setPower(yAxleMotion + rotationalMotion + xAxleMotion);
        driver.leftRear.setPower(yAxleMotion + rotationalMotion - xAxleMotion);
        driver.rightFront.setPower(yAxleMotion - rotationalMotion - xAxleMotion);
        driver.rightRear.setPower(yAxleMotion - rotationalMotion + xAxleMotion);

        if (gamepad.dpad_down & previousMotionModeButtonActivation.seconds() > 0.5 & !slowMotionModeSuggested) { // when control mode button is pressed, and hasn't been pressed in the last 0.3 seconds. pause this action when slow motion mode is already suggested
            slowMotionModeRequested = !slowMotionModeRequested; // activate or deactivate slow motion
            previousMotionModeButtonActivation.reset();
        } if(gamepad.dpad_up & previousNavigationModeButtonActivation.seconds() > 0.5) {
            groundNavigatingModeActivationSwitch = !groundNavigatingModeActivationSwitch;
            previousNavigationModeButtonActivation.reset();
        } if(gamepad.dpad_left & previousYAxleReverseSwitchActivation.seconds() > 0.5) {
            yAxleReversedSwitch = !yAxleReversedSwitch;
            previousYAxleReverseSwitchActivation.reset();
        } if (gamepad.dpad_right) { // debug the imu by resetting the heading
            // imu.resetYaw();
        }

        slowMotionModeActivationSwitch = slowMotionModeRequested | slowMotionModeSuggested; // turn on the slow motion mode if it is suggested by the system or if it is requested by the pilot

        dt.reset();
    }

    public void setSlowMotionModeActivationSwitch(boolean suggested) {
        this.slowMotionModeSuggested = suggested;
    }

    private double[] navigateGround(double objectiveXMotion, double objectiveYMotion, double facing) {
        double[] correctedMotion = new double[2];

        // correct the motion
        double speed = Math.sqrt(objectiveXMotion * objectiveXMotion + objectiveYMotion * objectiveYMotion); // the magnitude of resultant velocity

        int sector; // calculate the sector of the direction we want the robot to move(in reference to the ground)
        if (objectiveXMotion > 0) {
            if (objectiveYMotion > 0) sector = 1;
            else sector = 4;
        } else if (objectiveYMotion > 0) sector = 2;
        else sector = 3;

        // System.out.print(sector); System.out.print("    "); // until here, no problem

        objectiveXMotion = Math.abs(objectiveXMotion); objectiveYMotion = Math.abs(objectiveYMotion); // take abs value, avoid errors in calculation
        double targetedVelocityArc = Math.atan(objectiveYMotion / objectiveXMotion); // find the angle of the direction we want the robot to move, but taking it down to sector 1
        switch (sector) { // transfer targetedVeloctiyArc back to the sector its taken down from
            // at sector 1, the velocity arc is itself
            case 2 :{
                targetedVelocityArc = Math.PI - targetedVelocityArc; // at sector 2, the arc needs to be flipped according to y-axis, so the actual arc will be 180deg - arc
                break;
            }
            case 3 :{
                targetedVelocityArc += Math.PI; // at sector 3, the arc needs to be pointing at the other way around, makes the actual arc be 180deg + arc
                break;
            }
            case 4 :{
                targetedVelocityArc = Math.PI * 2 - targetedVelocityArc; // at sector 4, the arc needs to be flipped according to x-axis
                break;
            }
        }

        // System.out.print(targetedVelocityArc); System.out.print("    "); // until here, no problem found

        double correctedVelocityArc = targetedVelocityArc + facing; // correct the direction of velocity according the direction of the car, the corrected velocity is now in reference to the car itself.

        // System.out.print(correctedVelocityArc); System.out.print("    "); // no problem until here

        if (correctedVelocityArc > Math.PI) {
            // if (correctedVelocityArc > 3/2*Math.PI) {sector = 4; correctedVelocityArc = Math.PI*2 - correctedVelocityArc; } // error, 3/2 will be automatically taken down to integer
            if (correctedVelocityArc > Math.PI*1.5) {sector = 4; correctedVelocityArc = Math.PI*2 - correctedVelocityArc; } // taking it down to sector 1 is the reverse of the above process
            else {sector = 3; correctedVelocityArc -= Math.PI; }
            // } else if (correctedVelocityArc > 1/2*Math.PI) {sector = 2; correctedVelocityArc = Math.PI - correctedVelocityArc; } // error, 1/2 will be automatically taken down to integer
        } else if (correctedVelocityArc > Math.PI*0.5) {sector = 2; correctedVelocityArc = Math.PI - correctedVelocityArc; }
        else {sector = 1; } // judge the sector of correctVelocityArc, store the secotr, and take correctedVelocityArc down to sector1

        // System.out.print(correctedVelocityArc); System.out.print("    "); System.out.print(sector); System.out.print("    "); // problem found, sector incorrect

        double xVelocity = 0, yVelocity = 0; // the velocity the robot needs to achieve the resultant velocity, now in refernce to itself
        xVelocity = Math.cos(correctedVelocityArc) * speed;
        yVelocity = Math.sin(correctedVelocityArc) * speed;
        switch (sector) { // transform the velocity back to the sector
            // in the first sector, velocity remains the same
            case 2 :{
                xVelocity *= -1; // in the second sector, velocity is mirrored according to y-axis
                break;
            }
            case 3 :{
                xVelocity *= -1;
                yVelocity *= -1; // in the third sector, velocity is pointing to the opposite direction
                break;
            }
            case 4:{
                yVelocity *= -1; // in the fourth sector, velocity is mirrored according to x-axis
                break;
            }
        }

        correctedMotion[0] = xVelocity;
        correctedMotion[1] = yVelocity;

        return correctedMotion;
    }


    /**
     * rotate the robot to make it stick to the rotation where it's asked to be
     * if the pilot asks to rotate, just rotate
     * otherwise, the robot sticks to the rotation where it was during the last pilot command
     *
     * @param rotationalAttempt the rotational speed that the pilot inputs
     * @return the required rotational motor speed to make the robot stick the pilot's commands
     */
    public double getRotationMotorSpeed(double rotationalAttempt) {
        double rotationMotorSpeed;
        if (Math.abs(rotationalAttempt) > 0.05) { /* if the pilot asks the robot to rotate */
            /* update the robot's current position */
            targetedRotation = positionCalculator.getRobotRotation();
            /* do a linear map to shrink the motor speed into a set range */
            rotationMotorSpeed = Math.copySign(
                    linearMap(minStickValue, 1, minMovingMotorPower, maxMovingMotorPower, Math.abs(rotationalAttempt)),
                    rotationalAttempt
            );
        } else { /* if the pilot didn't do any rotation */
            /* find the closest path do the original rotation */
            double rotationDifference = AutoStageRobotChassis_tmp.reformatRotationDifference(
                    targetedRotation - positionCalculator.getRobotRotation());

            /* do a linear map to find out what motor speed should be given */
            rotationMotorSpeed  = Math.copySign(
                    linearMap(rotationTolerance,rotationStartsSlowingDown,minMovingMotorPower,maxMovingMotorPower,
                            Math.abs(rotationDifference)),
                    rotationDifference
            );

            rotationMotorSpeed *= encoderRotationToMotorSpeedFactor;
        }

        System.out.println(rotationMotorSpeed);

        return rotationMotorSpeed;
    }

    private double linearMap(double value) {
        if (slowMotionModeActivationSwitch) { // when slow motion activated
            if (value > 0) return linearMapMethod(minStickValue, 1, minDrivingPower, maxCarryingPower, value);
            return linearMapMethod(-1, -minStickValue, -maxCarryingPower, minDrivingPower, value); // change the speed range to -0.4~0.4
        } if (value > 0) return linearMapMethod(minStickValue, 1, minDrivingPower, maxDrivingPower, value);
        return linearMapMethod(-1, -minStickValue, -maxDrivingPower, minDrivingPower, value); // map the axle of the stick to make sure inputs below 10% are ignored
    }
    public static double linearMapMethod(double fromFloor, double fromCeiling, double toFloor, double toCeiling, double value){
        if (value > Math.max(fromCeiling, fromFloor)) return Math.max(toCeiling, toFloor);
        else if (value < Math.min(fromCeiling, fromFloor)) return Math.min(toCeiling, toFloor);
        value -= fromFloor;
        value *= (toCeiling-toFloor) / (fromCeiling - fromFloor);
        value += toFloor;
        return value;
    }

    public static double linearMap(double fromFloor, double fromCeiling, double toFloor, double toCeiling, double magnitude) {
        return Math.copySign(
                linearMapMethod(fromFloor, fromCeiling, toFloor, toCeiling, magnitude),
                magnitude
        );
    }

    public double getLastMovementTime() {
        return lastMovement.seconds();
    }
}