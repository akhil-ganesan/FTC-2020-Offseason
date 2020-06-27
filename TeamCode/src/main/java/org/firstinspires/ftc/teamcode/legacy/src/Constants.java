package org.firstinspires.ftc.teamcode.legacy.src;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public final class Constants {
    // ------------------
    //  Drive Subsystem
    // ------------------
    public static final String frontRight = "Front Right";
    public static final String backRight = "Back Right";
    public static final String frontLeft = "Front Left";
    public static final String backLeft = "Back Left";
    public static final double DRIVE_P = 5.0;
    public static final double turnScaling = 1;
    public static final double strafeScaling = 1.5;
    // --------------------------
    //  Vuforia Vision Constants
    // --------------------------
    public static VuforiaTrackables targetsSkyStone = null;
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;
    public static final String VUFORIA_KEY =
            "AWWCp8z/////AAABmQqV/K50N0OTqlyIYanMsyQ6huM5ckTKtdjF0/gyTwTINZPIGhLWxx3ag5PUmAw90BOHnZh3arwMSH0sjWZUM7wTJG/rcPmsj3MFp2eSPPc+osid/6jBjyg8YuhBYFN8jO3YvFlo/24qqX8K1DWOX8GU7dAfZEIhI71HCmY+pRWIGxKWyXxkpf3xULPPommaHqF7wSA/z37uQs+zSTs9SJKxiGvUlF7oYkVkURIuzovMKiK7rRqQT/dmCKH/JFpxgl8Er3O50/DL03EMmmNbjkiqA4vAU7wwD8rTkHympjAl7MnSmQRtXWxyRUildftpaQr7rD8vuz+4A6j/+/nKeTUanIi1fPMuE0Xa+Cth7SDr";
    private static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor
    // Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;
    // Center-Support Targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;
    // Perimeter Targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;
    // Phone Constants
    public static final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    public static float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
    // --------------------
    //  TF Vision Constants
    // --------------------
    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
}
