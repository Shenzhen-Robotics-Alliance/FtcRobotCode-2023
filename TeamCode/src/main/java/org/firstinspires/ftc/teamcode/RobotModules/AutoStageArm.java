package org.firstinspires.ftc.teamcode.RobotModules;

/**
 * the program that connects to the arm module to control the arm during auto stage
 *
 * @Deprecated
 */
@Deprecated
public class AutoStageArm {
    // TODO set these configurations
    /** the sets of positions of the arm to match the sleeves stack with different amount of sleeves left */
    private static final int[] sleevesStackPositions = {120,80,0,0};
    /** count the number of sleeves obtained already */
    private int sleevesCount = 0;
    Arm armModule;
    public AutoStageArm(Arm armModule) {
        this.armModule = armModule;
    }

    public void holdPreLoadedSleeve() {
        armModule.closeClaw();
        armModule.toGroundArmPosition();
        while (armModule.getArmStatusCode() <= 0) armModule.periodic();
    }

    public void dropSleeve() {
        armModule.deactivateArm();
        while (armModule.getArmStatusCode() <= -1) armModule.periodic();
        armModule.openClaw();
    }

    //public void

    public void goToHighestTower() {
        armModule.toHighArmPosition();
        while (armModule.getArmStatusCode() > 0) {
            armModule.periodic();
            System.out.println(armModule.getArmStatusCode());
        }
    }

    public void grabFromSleevesStack() {
        armModule.elevateArm(sleevesStackPositions[sleevesCount++]);
        while (armModule.getArmStatusCode() > 0) armModule.periodic();
    }

    public void liftFromSleevesStack() {
        armModule.closeClaw();
        while (armModule.getArmStatusCode() > 0) armModule.periodic();
        armModule.toLowArmPosition();
    }
}
