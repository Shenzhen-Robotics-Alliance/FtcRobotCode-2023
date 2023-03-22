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

import org.firstinspires.ftc.teamcode.HardwareDriver;
import org.firstinspires.ftc.teamcode.RobotModule;

import java.util.HashMap;

public class Arm extends RobotModule {
    /** highest position of the arm */
    private final int highPos = 700;
    /** midpoint position of the arm */
    private final int midPos = 450;
    /** lower position of the arm */
    private final int lowPos = 280;
    /** loading position of the arm */
    private final int gndPos = 45;

    /** connects to the hardware */
    private HardwareDriver hardwareDriver;

    /** connects to the game pad that's used to control the arm */
    private Gamepad gamepad;

    /** whether the claw is opened */
    private boolean claw;

    /** the position code for the arm
    *  -1: the arm is relaxed
    *   0: the arm is attached to the ground to capture sleeves,
    *   1: the arm is at the position matching the lowest tower,
    *   2: the arm is at the position matching the middle tower,
    *   3: the arm is at the position matching the highest tower,
    */
    private short armPositionCode;

    /**
     * the status code for the arm
     * -1: the arm is relaxed and waiting for pilot's instructions;
     * 0: the arm is holding still at current height and waiting for pilot's instructions;
     * 1: the arm is motioning downwards and has a distance between the objective position, so it should continue moving downwards to go to the targeted position;
     * 2: the arm is motioning upwards and has a distance between the objective position, so it should continue moving upwards to go to the targeted position;
     * 3: the arm is currently motioning downwards but is already close to the objective position, so it should decelerate;
     * 4: the arm is currently motioning upwards but is already close to the objective position, so it should immediately jump to status 0 and maintain height, after being slowed down due to gravity;
     * 5: the arm is doing the complete process of grabbing and is current closing it's claw, 300ms after being so the should then be lifted to middle position
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

    /** the chassis module of robot */
    private RobotChassis robotChassis;

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
     */
    @Override
    public void init(
            HashMap<String, RobotModule> dependentModules,
            HashMap<String, Object> dependentInstances
    ) throws NullPointerException {
        /* throw out an error if the dependent module is given an empty map */
        if (dependentModules.isEmpty()) throw new NullPointerException(
                "an empty map of dependent modules given to this module, which requires at least one modular dependencies"
        );

        /* throw out an error if the dependentInstances is given an empty map */
        if (dependentInstances.isEmpty()) throw new NullPointerException(
                "an empty map of dependent instances given to this module, which requires at least one instant dependencies"
        );

        /* get the dependent modules from the param */
        if (! dependentModules.containsKey("robotChassis")) throw new NullPointerException(
                "dependent module not given: " + "robotChassis"
        );
        this.robotChassis = (RobotChassis) dependentModules.get("robotChassis");

        /* get the instances from the param */
        if (! dependentInstances.containsKey("hardwareDriver")) throw new NullPointerException(
                "dependent instance not given: " + "hardwareDriver"
        );
        this.hardwareDriver = (HardwareDriver) dependentInstances.get("hardwareDriver");
        if (! dependentInstances.containsKey("initialControllerPad")) throw new NullPointerException(
                "dependent instance not given: " + "initialControllerPad"
        );
        this.gamepad = (Gamepad) dependentInstances.get("initialControllerPad");

        /* set the robot's arm to be the default status */
        hardwareDriver.claw.setPosition(0.6);
        this.claw = false;
        deactivateArm();
        this.armPositionCode = -1;
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
        switch (armStatusCode) {
            case -1: case 0: {
                reactToPilotInputs();
                break;
            }  case 1: {
                waitForDeclinedCompletion();
                break;
            } case 2: {
                waitForInclinedCompletion();
                break;
            } case 3: {
                waitForDecelerateCompletion();
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
    private void reactToPilotInputs() {
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
        if (gamepad.right_trigger>0.2 & PreviousGrepActivation.seconds() > .3) {
            PreviousGrepActivation.reset();
            this.openClaw();
            this.deactivateArm();
            // TODO aim the target automatically using computer vision
            this.closeClaw();

            this.toMidArmPosition();
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

        if (PreviousElevatorActivation.seconds() > 30 & robotChassis.getLastMovementTime() > 30 & PreviousClawActivation.seconds() > 30) { // no operation after 30s
            hardwareDriver.lift_left.setPower(0);
            hardwareDriver.lift_left.setPower(0);
            System.out.println("saving battery...");
            System.exit(0);
        } if (PreviousElevatorActivation.seconds() > 5 & this.getClaw()) {
            System.out.println("cooling down the motors...");
            this.deactivateArm(); // deactivate when no use for 5 seconds so that the motors don't overheat
            PreviousElevatorActivation.reset(); // so that it does not proceed deactivate all the time
        }

        // control slow motion automatically
        if (this.getArmIsBusy()) robotChassis.setSlowMotionModeActivationSwitch(true);
        else robotChassis.setSlowMotionModeActivationSwitch(false);
    }

    /**
     * called when status code is 1,
     * meaning the arm is currently moving downward and should decelerate when the it's close to the objective position
     */
    private void waitForDeclinedCompletion() {
        if (
                Math.abs(hardwareDriver.lift_left.getCurrentPosition()-targetedArmPosition) < 20 |
                        Math.abs(hardwareDriver.lift_right.getCurrentPosition()-targetedArmPosition) < 20)
        {
            /* set the arms to be decelerating */
            hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardwareDriver.lift_right.setVelocity(0);
            hardwareDriver.lift_left.setVelocity(0);

            /* update the status code of the arm telling that they are currently working to slow down */
            armStatusCode = 3;
        }
    }

    /**
     * called when status code is 2,
     * meaning the arm is currently moving upward and should jump to maintenance of height the moment it gets close enough to the objective position
     */
    private void waitForInclinedCompletion() {
        /* wait until the movement is completed */
        if (
                Math.abs(hardwareDriver.lift_left.getCurrentPosition()-targetedArmPosition) < 5 |
                        Math.abs(hardwareDriver.lift_right.getCurrentPosition()-targetedArmPosition) < 5)
        {
            /* when the movement is completed, set the arm to stay still */
            hardwareDriver.lift_left.setTargetPosition(targetedArmPosition);
            hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardwareDriver.lift_right.setTargetPosition(targetedArmPosition);
            hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* update the status code of the arm telling that they are maintaining height at current position */
            armStatusCode = 0;
        }
    }

    /**
     * called when the status code is 3,
     * meaning the arm is currently motioning downwards but is already close to the objective position,
     * so it should slow down to avoid hitting the structure of the robot heavily
     */
    private void waitForDecelerateCompletion() {
        /* wait until the slow-down is completed, accept any deviation less than 10 */
        if (Math.abs(hardwareDriver.lift_left.getVelocity()) < 10) {
            /*  make the motor stick in their positions */
            hardwareDriver.lift_left.setTargetPosition(targetedArmPosition);
            hardwareDriver.lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardwareDriver.lift_right.setTargetPosition(targetedArmPosition);
            hardwareDriver.lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* update the status code of the arm telling that the slow-down is completed and they are maintaining height at current position  */
            armStatusCode = 0;
        }
    }

    /**
     * move the arm down into the following lower level
     */
    public void lowerArm() {
        System.out.println(this);
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
        elevateArm(highPos);
        armPositionCode = 3;
        armIsBusy = true;
    }
    /**
     * set the arm to match the higher, middle and lowest tower
     */
    public void toMidArmPosition() {
        elevateArm(midPos);
        armPositionCode = 2;
        armIsBusy = true;
    }
    /**
     * set the arm to match the higher, middle and lowest tower
     */
    public void toLowArmPosition() {
        elevateArm(lowPos);
        armPositionCode = 1;
        armIsBusy = true;
    }

    public void toGroundArmPosition() {
        elevateArm(gndPos);
        armPositionCode = 0;
        armIsBusy = false;
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
        armPositionCode = 1;
        armIsBusy = false;
    }

    /**
     * move the arm to the targeted position
     * when the arm is almost there, reverse the motors and decelerate to avoid damage to the structer
     * exit the function when the arm is close enough to the objective
     *
     * @param position: the targeted position, ranged 0-1000, 0 is when the arm hits the robot badly, 1000 is when the arm flips around and damage the structer
     */
    private void elevateArm(int position) {
        /** the direction that the arm is going
        *   true when the arm is going up
        *   false when the arm is going down
        */
        boolean isDecline = position < hardwareDriver.lift_left.getCurrentPosition();

        if (isDecline) {
            /* set the power of the motor
             * 20% when it's going down, in considerate of the impulse of gravitation
             * */
            double armDeclineSpeed = 0.2;
            hardwareDriver.lift_left.setPower(armDeclineSpeed);
            hardwareDriver.lift_right.setPower(armDeclineSpeed);

            /* if the arm is declining
             * set the status code so the arm will decelerate in future calls of periodic
             * */
            this.armStatusCode = 1;
        } else {
            /* set the power of the motor
             * 40% when it's going up
             * */
            double armInclineSpeed = 0.4;
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

    public void deactivateArm() {
        /* if (
                arm == -1 |
                (!claw) // if the claw is set to be closed
        ) return; // if the arm is already deactivated, or if the claw is holding stuff, abort */
        while (armPositionCode > 0) lowerArm(); // put the arm down step by step
        openClaw();
        hardwareDriver.lift_left.setPower(0);
        hardwareDriver.lift_right.setPower(0);
        armPositionCode = -1;
        targetedArmPosition = armPositionCode;
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
        System.out.println("opening");
        claw = true;
        hardwareDriver.claw.setPosition(.35); // open grabber
        // while (Math.abs(hr.claw.getPosition() - .35) > .05) Thread.yield(); // wait until the movement is finished, accept any inaccuracy below 5%
        armIsBusy = false;
    }
    /**
     * close the claw of the arm by setting the position of the servo driving it
     */
    public void closeClaw() {
        System.out.println("closing");
        claw = false;
        hardwareDriver.claw.setPosition(.65); // close grabber
        // while (Math.abs(hr.claw.getPosition() - .65) > .05) Thread.yield();
        armIsBusy = true;
    }

    public boolean getClaw() {
        return claw;
    }

    public boolean getArmIsBusy() {
        /* true: busy, false: free */
        return armIsBusy;
    }


}
