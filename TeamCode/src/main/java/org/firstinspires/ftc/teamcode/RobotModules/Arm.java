/*
 * Copyright © 2023 SCCSC-Robotics-Club
 * FileName: ArmControllingMethods.java
 *
 * the program that controls the elevation of the arm and operations of the claw
 *
 * @Author 四只爱写代码の猫
 * @Date 2023.2.27
 * @Version v0.1.0
 * */
package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivers.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

import dalvik.system.DelegateLastClassLoader;

public class Arm extends RobotModule {
    /** highest position of the arm */
    private int highPos = 745;
    /** midpoint position of the arm */
    private int midPos = 480;
    /** lower position of the arm */
    private int lowPos = 320;
    /** loading position of the arm */
    private int gndPos = 65;
    /**
     * power of the motor to lower the arm
     * 20% when it's going down, in considerate of the impulse of gravitation
     */
    double armDeclineSpeed = 0.2;
    /**
     * power of the motor to lower the arm
     * 40% when it's going up
     */
    double armInclineSpeed = 0.4;

    /** connects to the hardware */
    private HardwareDriver hardwareDriver;

    /** connects to the game pad that's used to control the arm */
    private Gamepad gamepad;

    /** whether the claw is closed */
    private boolean claw;

    /** the position code for the arm
    *  -1: the arm is relaxed
    *   0: the arm is attached to the ground to capture sleeves,
    *   1: the arm is at the position matching the lowest tower,
    *   2: the arm is at the position matching the middle tower,
    *   3: the arm is at the position matching the highest tower,
    */
    public short armPositionCode;

    /**
     * the status code for the arm
     *
     * -1: the arm is relaxed and waiting for pilot's instructions;
     * 0: the arm is holding still at current height and waiting for pilot's instructions;
     * 1: the arm is motioning downwards and has a distance between the objective position, so it should continue moving downwards to go to the targeted position;
     * 2: the arm is motioning upwards and has a distance between the objective position, so it should continue moving upwards to go to the targeted position;
     * 3: the arm is currently motioning downwards but is already close to the objective position, so it should decelerate;
     * 4: the arm is currently motioning upwards but is already close to the objective position, so it should immediately jump to status 0 and maintain height, after being slowed down due to gravity;
     * 5: the arm is doing the complete process of grabbing and is currently on it's way to the lowest position
     * 6: the arm is doing the complete process of grabbing and is current closing it's claw, 300ms after being so the should then be lifted to middle position
     * 7: the arm is in the deactivation process and is moving downward, it should jump out and go to code -1 after it reaches ground
     */
    private short armStatusCode;

    /** the status of the arm
    *   true: the arm is busy and in use
    *   false: the arm is free, no operation is proceeding
    */
    private boolean armIsBusy;

    /**
     * the targeted encoder position of the arm, ranged 0-1000, -1 for relaxed;
     */
    private int targetedArmPosition = -1;

    /** variables that record the time after pressing a button, so that the button is not activated over and over */
    private final ElapsedTime PreviousElevatorActivation = new ElapsedTime();
    private final ElapsedTime PreviousClawActivation = new ElapsedTime();
    private final ElapsedTime PreviousGrepActivation = new ElapsedTime();

    /** to make sure the robot waits until the grabbing is finished */
    private final ElapsedTime lastGrabbingDelay = new ElapsedTime();

    /** time when the claw opens, so that RAS does not step in too fast */
    private final ElapsedTime lastOpenTime = new ElapsedTime();

    /** the chassis module of robot */
    private PilotChassis pilotChassis;
    /** whether to inform the robot chassis to slow down */
    private boolean inManualStage;

    /**
     * construct function of arm controlling methods
     * set the module's name to be "Arm"
     *
     */
    public Arm() {
        super("Arm");
    }

    /**
     * initialize the arm module
     * given all the instances that connects to the robot's hardware
     *
     * @param dependentModules this module needs the following modules(pass them in the form of hashmap):
     *                         "robotChassis" : RobotChassis, the module that controls the chassis of the robot
     * @param dependentInstances this module needs the following instances(pass them in the form of hashmap):
     *                           "hardwareDriver" : HardwareDriver, the driver that connects to the hardware, gained from super class "LinearOpMode"
     *                           "initialControllerPad" : com.qualcomm.robotcore.hardware.Gamepad, the default game pad used to control the robot's arm
     * @param informRobotChassis  whether to inform the robot chassis to go to slow-motion mode automatically
     */
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances,
            boolean informRobotChassis
    ) throws NullPointerException {
        this.pilotChassis = null;
        if (informRobotChassis) {
            /* throw out an error if the dependent module is given an empty map */
            if (dependentModules.isEmpty()) throw new NullPointerException(
                    "an empty map of dependent modules given to this module, which requires at least one modular dependencies"
            );
            /* get the dependent modules from the param */
            if (! dependentModules.containsKey("robotChassis")) throw new NullPointerException(
                    "dependent module not given: " + "robotChassis"
            );
            this.pilotChassis = (PilotChassis) dependentModules.get("robotChassis");
        }

        this.inManualStage = informRobotChassis;

        /* throw out an error if the dependentInstances is given an empty map */
        if (dependentInstances.isEmpty()) throw new NullPointerException(
                "an empty map of dependent instances given to this module, which requires at least one instant dependencies"
        );

        /* get the instances from the param */
        if (! dependentInstances.containsKey("hardwareDriver")) throw new NullPointerException(
                "dependent instance not given: " + "hardwareDriver"
        );
        this.hardwareDriver = (HardwareDriver) dependentInstances.get("hardwareDriver");
        if (! dependentInstances.containsKey("initialControllerPad")) throw new NullPointerException(
                "dependent instance not given: " + "initialControllerPad"
        );
        this.gamepad = (Gamepad) dependentInstances.get("initialControllerPad");


        /* calibrate the arm according to the starting positions */
        double startingPos = hardwareDriver.lift_left.getCurrentPosition();
        highPos += startingPos; midPos += startingPos; lowPos += startingPos;

        /* set the robot's arm to be the default status */
        openClaw();
        this.claw = false;
        deactivateArm();
        this.armPositionCode = -1;

    }

    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) {
        init(dependentModules, dependentInstances, true);
    }

    /**
     * update the hardware port or controller pad of the arm function
     *
     * @param instanceName the name of instance to update
     *                     "hardwareDriver" : the connection to the robot's hardware,
     *                     "controllerPad" : the game pad used to control the robot
     * @param newerInstance the newer instance to replace the older one
     * @throws NullPointerException if an none-exist instance is selected
     */
    @Override
    public void updateDependentInstances(String instanceName, Object newerInstance) throws NullPointerException {
        switch (instanceName) {
            case "hardwareDriver" : {
                this.hardwareDriver = (HardwareDriver) newerInstance;
                break;
            }
            case "controllerPad" : {
                this.gamepad = (Gamepad) newerInstance;
                break;
            }
            default: throw new NullPointerException("attempting to update a none-exist instance");
        }
    }

    @Override
    public void periodic() {
        /* no mater what, respond to the pilot's input first, so that the pilots have the control over their machine and can interrupt actions */
        reactToPilotInputs();
        /* proceed different instructions according to the status code */
        switch (armStatusCode) {
            case -1: {
                /* save battery and cool down the arms */
                hardwareDriver.lift_left.setPower(0);
                hardwareDriver.lift_right.setPower(0);
                break;
            } case 0: {
                setArmStill();
                if (inManualStage) powerSavingAndChassisStrategy();
                break;
            }
            case 1: {
                waitForDeclinedCompletion((short) 0);
                break;
            } case 2: {
                waitForInclinedCompletion();
                break;
            } case 3: {
                waitForDecelerateCompletion();
                break;
            } case 5: {
                /* jump out of this status when the arm is almost on the ground */
                if (hardwareDriver.lift_left.getCurrentPosition() - gndPos < 30 || hardwareDriver.lift_right.getCurrentPosition() - gndPos < 30) {
                    /* set a 300ms delay to closing the claw */
                    lastGrabbingDelay.reset();
                    armStatusCode = 6;
                }
                break;
            }
            case 6: {
                if (lastGrabbingDelay.seconds() > 0.3) {
                    closeClaw();
                    this.toLowArmPosition(); // raise the arm up
                }
                break;
            } case 7: {
                /* wait for the arm to move to the ground */
                waitForDeclinedCompletion((short) -1);
                break;
            }
        }
    }

    /**
     * reacts to the instructions of the pilot
     * called when status code is -1 or 0,
     * meaning the robot is still and waiting for commands
     *
     * configuration of the game pad:
     *      right_bumper: close the claw to grab stuffs
     *      left_bumper: open the claw to release stuffs
     *      button_Y: raise the arm to the position matching the highest tower
     *      button_X: raise the arm to the position matching the highest tower
     *      button_B: raise the arm to the position matching the highest tower
     *      button_A: raise the arm to the position matching the highest tower
     *      right_trigger: proceed a complete movement that opens the claw, moves the arm to the ground, grab the sleeve, and lift it up (if not activated during the past 200ms)
     *      left_stick: move the arm up/down to the match next higher/lower tower (if not activated in the past 200ms)
     * configuration of the auto-switching control mode:
     *      when the claw is closed or when the arm is raised, slow motion mode is switched on automatically
     *      if the robot's arm is not holding stuff but have been hanging up for more than 5 seconds, it deactivates automatically so the motors can cool down
     *      when the robot have been staying still without any driver inputs, it saves battery by turing off the motors and aborting the main program
     */
    private void  reactToPilotInputs() {
        /* controls the opening and closing of the claw to grab stuff */
        if (gamepad.right_bumper) closeClaw();
        else if (gamepad.left_bumper) this.openClaw();

        if (gamepad.y) {
            this.toHighArmPosition();
        }
        if (gamepad.x) {
            this.toMidArmPosition();
        }
        if (gamepad.b) {
            this.toLowArmPosition();
        }
        if (gamepad.a) {
            this.toGroundArmPosition();
        }

        if (gamepad.left_stick_y < -0.8 & PreviousElevatorActivation.seconds() > .3) { // the elevator cannot be immediately activated until 0.2 seconds after the last activation
            System.out.println("RA");
            this.raiseArm();
            PreviousElevatorActivation.reset();
        } else if (gamepad.left_stick_y > 0.8 & PreviousElevatorActivation.seconds() > .3) {
            System.out.println("LA");
            this.lowerArm();
            PreviousElevatorActivation.reset();
        }
    }

    /**
     * TODO edit explanations
     * */
    private void powerSavingAndChassisStrategy() {
        if (PreviousElevatorActivation.seconds() > 30 & pilotChassis.getLastMovementTime() > 30 & PreviousClawActivation.seconds() > 30) { // no operation after 30s
            hardwareDriver.lift_left.setPower(0);
            hardwareDriver.lift_left.setPower(0);
            System.out.println("saving battery...");
            System.exit(0);
        }
        if (PreviousElevatorActivation.seconds() > 1.5 & !this.getClaw()) {
            System.out.println("cooling down the motors...");
            this.deactivateArm(); // deactivate the arm to avoiding burning the motors
            PreviousElevatorActivation.reset(); // so that it does not proceed deactivate all the time
        }

        // control slow motion automatically
        if (this.getArmIsBusy()) pilotChassis.setSlowMotionModeActivationSwitch(true);
        else pilotChassis.setSlowMotionModeActivationSwitch(false);
    }

    /**
     * let the arm maintain it's current height
     */
    private void setArmStill() {
        /* if the encoders of the arms are working well */
        if (hardwareDriver.lift_left.getCurrentPosition() != 0 && hardwareDriver.lift_right.getCurrentPosition() != 0) {
            hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardwareDriver.lift_right.setTargetPosition(targetedArmPosition);
            hardwareDriver.lift_left.setTargetPosition(targetedArmPosition);
            return;
        }

        /* use only one encoder to determine how the motors spin */
        System.out.println("<-- arm encoder not functioning --> ");
        hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* get the current position, velocity and power of the arm according to the more reliable motor */
        double currentPosition = Math.max(hardwareDriver.lift_left.getCurrentPosition(), hardwareDriver.lift_right.getCurrentPosition());
        double currentVelocity; if (hardwareDriver.lift_right.getVelocity() != 0) currentVelocity = hardwareDriver.lift_right.getVelocity(); else currentVelocity = hardwareDriver.lift_left.getVelocity();
        double currentPower = Math.max(hardwareDriver.lift_left.getPower(), hardwareDriver.lift_right.getPower());

//        /* the power that the motors need in order to fall */
//        double fallingPower; if (currentPosition > 600) fallingPower = 0; else fallingPower = -0.1;
//        /* the power that the motors need to  */

        /** the amount of motor power the system adjusts in every period, when it's running like what we want */
        final double powerAttemptingDifference = 0.02;
        double armPower = 0;
//        if (targetedArmPosition-currentPosition < -20) {
//            /* decline the arms */
//            final double armDecliningVelocity = -300;
//            if (currentVelocity - armDecliningVelocity < -20) armPower = currentPower + powerAttemptingDifference;
//            else if(currentVelocity - armDecliningVelocity > 20) armPower = currentPower - powerAttemptingDifference;
//        } else if (targetedArmPosition-currentPosition > 20) {
//            /* raise the arms */
//            final double armIncliningVelocity = 300;
//            if (currentVelocity - armIncliningVelocity < -20) armPower = currentPower + powerAttemptingDifference;
//            else if(currentVelocity - armIncliningVelocity > 20) armPower = currentPower - powerAttemptingDifference;
//        } else {
//            /* set the arms to be still */
//            if (currentVelocity < -20) armPower = currentPower + powerAttemptingDifference;
//            else if(currentVelocity > 20) armPower = currentPower - powerAttemptingDifference;
//        }
        if (currentVelocity < -20) armPower = currentPower + powerAttemptingDifference;
        else if(currentVelocity > 20) armPower = currentPower - powerAttemptingDifference;
        System.out.println(currentVelocity + ", " + armPower);

        /* set the motor speed(almost forgot) */
//        hardwareDriver.lift_right.setPower(armPower);
//        hardwareDriver.lift_left.setPower(armPower);
    }

    /**
     * called when status code is 1,
     * meaning the arm is currently moving downward and should decelerate when the it's close to the objective position
     */
    private void waitForDeclinedCompletion(short statusCodeAfterCompletion) {
        if (Math.abs(hardwareDriver.lift_left.getCurrentPosition() - targetedArmPosition) < 200) {
            /* set the arms to be decelerating */
            hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardwareDriver.lift_right.setVelocity(0);
            hardwareDriver.lift_left.setVelocity(0);

            /* update the status code of the arm telling that they are currently working to slow down */
            // armStatusCode = 3;

            /* just stop the process, forget about the deceleration */
            armStatusCode = statusCodeAfterCompletion;
        }
    }

    /**
     * called when status code is 2,
     * meaning the arm is currently moving upward and should jump to maintenance of height the moment it gets close enough to the objective position
     */
    private void waitForInclinedCompletion() {
        /* wait until the movement is completed */
        if (
                Math.abs(
                        Math.max(hardwareDriver.lift_left.getCurrentPosition(), hardwareDriver.lift_right.getCurrentPosition())
                                - targetedArmPosition) < 20
        ) {
            /* update the status code of the arm telling that they are maintaining height at current position */
            armStatusCode = 0;
            /* do periodic immediately */
            this.periodic();
        }
    }

    /**
     * called when the status code is 3,
     * meaning the arm is currently motioning downwards but is already close to the objective position,
     * so it should slow down to avoid hitting the structure of the robot heavily
     */
    private void waitForDecelerateCompletion() {
        hardwareDriver.lift_left.setVelocity(0);
        hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwareDriver.lift_right.setVelocity(0);
        hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* wait until the slow-down is completed, accept any deviation less than 10
        * just do the whole process inside one period, it does not take much time */
        while (
                Math.abs(hardwareDriver.lift_left.getVelocity()) > 15) {
            double power = Math.min(
                    0.2,
                    Math.abs(hardwareDriver.lift_left.getVelocity()) / 2000
            );
        }

        /* update the status code of the arm telling that the slow-down is completed and they are maintaining height at current position  */
        armStatusCode = 0;
        /* do periodic immediately */
        this.periodic();
    }

    /**
     * move the arm down into the following lower level
     */
    public void lowerArm() {
        switch (armPositionCode) {
            case 3: toMidArmPosition(); break;
            case 2: toLowArmPosition(); break;
            case 1: toGroundArmPosition(); break;
            case 0: deactivateArm(); break;
        }
    }

    /**
     * move the arm up into the higher neighboured level
     */
    public void raiseArm() {
        switch (armPositionCode) {
            case -1: case 0: toLowArmPosition(); break;
            case 1: toMidArmPosition(); break;
            case 2: toHighArmPosition(); break;
        }
    }

    /**
     * set the arm to match the higher tower
     */
    public void toHighArmPosition() {
        armPositionCode = 3;
        armIsBusy = true;
        elevateArm(highPos);
    }
    /**
     * set the arm to match the higher, middle and lowest tower
     */
    public void toMidArmPosition() {
        armPositionCode = 2;
        armIsBusy = true;
        elevateArm(midPos);
    }
    /**
     * set the arm to match the higher, middle and lowest tower
     */
    public void toLowArmPosition() {
        armPositionCode = 1;
        armIsBusy = true;
        elevateArm(lowPos);
    }

    public void toGroundArmPosition() {
        armPositionCode = 0;
        armIsBusy = false;
        elevateArm(gndPos);
    }

    /**
     * move the lowest position of the arm to load sleeves
     *
     * @Deprecated this method is longer suggested, use deactivateArm() instead
     */
    @Deprecated
    public void toLoadingArmPosition() {
        // go to the arm position to load the sleeves
        int loadingPos = 360;
        elevateArm(loadingPos);
        armPositionCode = 0;
        armIsBusy = false;
    }

    /**
     * move the arm to the targeted position
     * when the arm is almost there, reverse the motors and decelerate to avoid damage to the structer
     * exit the function when the arm is close enough to the objective
     *
     * @param position: the targeted position, ranged 0-1000, 0 is when the arm hits the robot badly, 1000 is when the arm flips around and damage the structer
     */
    public void elevateArm(int position) {
        /** the direction that the arm is going
        *   true when the arm is going up
        *   false when the arm is going down
        */
        boolean isDecline = position < hardwareDriver.lift_right.getCurrentPosition();

        if (isDecline) {
            /* set the power of the motor */
            hardwareDriver.lift_left.setPower(armDeclineSpeed);
            hardwareDriver.lift_right.setPower(armDeclineSpeed);

            /* if the arm is declining
             * set the status code so the arm will decelerate in future calls of periodic
             * */
            this.armStatusCode = 1;
        } else {
            /* set the power of the motor */
            hardwareDriver.lift_left.setPower(armInclineSpeed);
            hardwareDriver.lift_right.setPower(armInclineSpeed);

            /* if the arm isn't declining
             *  move to the position directly, as gravity will slow the arm down
             * */
            this.armStatusCode = 2;
        }

        /* set the arms to be moving, towards the objective position */
        hardwareDriver.lift_left.setTargetPosition(position);
        hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwareDriver.lift_right.setTargetPosition(position);
        hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* update the targeted position */
        this.targetedArmPosition = position;
    }

    public void armDeactivation() {
        /* wait until the arm goes below the lowest position */
        if (hardwareDriver.lift_left.getCurrentPosition() > gndPos) return;
        hardwareDriver.lift_left.setPower(0);
        hardwareDriver.lift_right.setPower(0);
        openClaw();
        armStatusCode = -1;
        armPositionCode = -1;
    }

    public void deactivateArm() {
        /* set the arm to move down */
        hardwareDriver.lift_left.setTargetPosition(gndPos);
        hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwareDriver.lift_right.setTargetPosition(gndPos);
        hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* update the targeted position */
        this.targetedArmPosition = gndPos;

        armStatusCode = 7;
        armPositionCode = -1;
    }


    /**
     * if the claw is opened, close it
     * if the claw is closed, open it
     *
     * @Deprecated this way of controlling is a bit confusing
     */
    @Deprecated
    public void open_closeClaw() {
        System.out.println("open_close");
        if(claw) {closeClaw(); return;}
        openClaw();
    }

    /**
     * open the claw of the arm by setting the position of the servo driving it
     */
    public void openClaw() {
        // System.out.println("opening");
        claw = false;
        hardwareDriver.claw.setPosition(0.45); // open grabber
        armIsBusy = false;
        lastOpenTime.reset();
    }
    /**
     * close the claw of the arm by setting the position of the servo driving it
     */
    public void closeClaw() {
        // System.out.println("closing");
        claw = true;
        hardwareDriver.claw.setPosition(0.1); // close grabber
        armIsBusy = true;
    }

    public boolean getClaw() {
        return claw;
    }

    public boolean getArmIsBusy() {
        /* true: busy, false: free */
        return armIsBusy;
    }

    /**
     * get the status code of the arm
     *
     * -1: the arm is relaxed and waiting for pilot's instructions;
     * 0: the arm is holding still at current height and waiting for pilot's instructions;
     * 1: the arm is motioning downwards and has a distance between the objective position, so it should continue moving downwards to go to the targeted position;
     * 2: the arm is motioning upwards and has a distance between the objective position, so it should continue moving upwards to go to the targeted position;
     * 3: the arm is currently motioning downwards but is already close to the objective position, so it should decelerate;
     * 4: the arm is currently motioning upwards but is already close to the objective position, so it should immediately jump to status 0 and maintain height, after being slowed down due to gravity;
     * 5: the arm is doing the complete process of grabbing and is currently on it's way to the lowest position
     * 6: the arm is doing the complete process of grabbing and is current closing it's claw, 300ms after being so the should then be lifted to middle position
     * 7: the arm is in the deactivation process and is moving downward, it should jump out and go to code -1 after it reaches ground
     */
    public short getArmStatusCode() {
        return armStatusCode;
    }

    /** get the time elapsed after the claw being closed, to debug RAS */
    public double getLastOpenTime() { return lastOpenTime.seconds(); }
}
