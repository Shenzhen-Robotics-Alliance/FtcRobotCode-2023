package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class ComputerVisionAUX {
    private final VuforiaTrackables relicTrackables;
    private final VuforiaTrackable relicTemplate;
    private final VuforiaLocalizer vuforia;
    private final WebcamName webcamName;
    public ComputerVisionAUX(HardwareMap hardwareMap) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );

        parameters.vuforiaLicenseKey = "AUrp8E//////AAABmR9EFFA6dECthZSIH4YjVNY8QqiA+WhfTqkO4koNt+QYbXrW9k3nqcvIL16tY13jQdTckP3BXh9+vKLJHQjATt6uffiXKfKUM3k+q3ZcB5g8v8+FT1WKJaOIf+vgOl84gghjBgira8FLQwuOB62wzkJRpoMKSNoyunOqG3PR7ttCl0izt9VRfaJy/2CmqLkXNj//tjcRj1xRqf3Xpo9omrSQcC8qQIRapKCcFUWyc+86WzQD2uVRwBAicmIs6yAHvUqMh/Xj0waomLYgvZ4CTDhgX2srIEgnLmlzxxIy2bggLV7vxnFILfe3wzLh1HYuQ0mHV9YF8ShgxXEbCrQDmUwripCsh3IST2Oxk/ZyVGZD";
        parameters.cameraName = webcamName;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        this.relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("cone"); // for debugging
    }

    public void activateAUX() {
        relicTrackables.activate();
    }

    public void aimCone() {
        AimCone aimCone = new AimCone();

    }
}

class AimCone implements Runnable {
    @Override
    public void run() {

    }
}
