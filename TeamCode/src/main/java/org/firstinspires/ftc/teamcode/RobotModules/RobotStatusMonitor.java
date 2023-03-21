package org.firstinspires.ftc.teamcode.RobotModules;

public class RobotStatusMonitor {
}

/** TODO write this file to replace
 *             Thread robotStatusMonitoringThread = new Thread(new Runnable() {
 *             @Override
 *             public void run() {
 *                 while (opModeIsActive() && !isStopRequested()) {
 *                     try {
 *                         Thread.sleep(100);
 *                     } catch (InterruptedException e) { throw new RuntimeException(e); }
 *                     System.out.println("monitoring thread running");
 *                     double[] robotCurrentPosition = fieldNavigation.getRobotPosition();
 *                     String cameraPositionString = String.valueOf(robotCurrentPosition[0]) + " " + String.valueOf(robotCurrentPosition[1]) + " " + String.valueOf(robotCurrentPosition[2]);
 *                     telemetry.addData("robotCurrentPosition(Camera)", cameraPositionString);
 *                     double[] encoderPosition = autoStageRobotChassis.getEncoderPosition();
 *                     String encoderPositionString = String.valueOf(encoderPosition[0]) + "," + String.valueOf(encoderPosition[1]);
 *                     telemetry.addData("robotCurrentPosition(Encoder)", encoderPositionString);
 *                     double encoderRotation = autoStageRobotChassis.getEncoderRotation();
 *                     telemetry.addData("robotCurrentRotation(Encoder)", encoderRotation);
 *                     telemetry.update();
 *                 }
 *             }
 *         }); // robotStatusMonitoringThread.start();
 */