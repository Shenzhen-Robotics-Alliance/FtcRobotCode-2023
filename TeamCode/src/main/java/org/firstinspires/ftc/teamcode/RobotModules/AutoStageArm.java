package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * the program that connects to the arm module to control the arm during auto stage
 *
 * @Deprecated
 */
@Deprecated
public class AutoStageArm {
    // TODO set these configurations
    /** the sets of positions of the arm to match the sleeves stack with different amount of sleeves left */
    private static final int[] sleevesStackPositions = {135,110,90,0};
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

    public void levelArmToSleevesStack() {
        armModule.openClaw();
        armModule.elevateArm(sleevesStackPositions[sleevesCount++]);
        while (armModule.getArmStatusCode() > 0) armModule.periodic();
    }

    public void liftFromSleevesStack() throws InterruptedException {
        armModule.closeClaw();
        Thread.sleep(300);
        armModule.toLowArmPosition();
        ElapsedTime timeUsed = new ElapsedTime(); timeUsed.reset();
        while (armModule.getArmStatusCode() > 0 && timeUsed.seconds() < 0.5) armModule.periodic();
    }

    public boolean getClaw() { return armModule.getClaw(); }
}
