package org.firstinspires.ftc.teamcode.team18103.src;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public final class Constants {
    // ------------------
    //  Drive Subsystem
    // ------------------
    public static final String frontRight = "Front Right";
    public static final String backRight = "Back Right";
    public static final String frontLeft = "Front Left";
    public static final String backLeft = "Back Left";
    public static final double turnScaling = 1;
    public static final double strafeScaling = 1.5;
    public static final double Dt = 0.01d;
    // Kauai Labs NavX Sensor
    public static final int NAVX_DIM_I2C_PORT = 0;
    public static final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public static final double COLLISION_THRESHOLD_DELTA_G = 0.5;
    public static final double TOLERANCE_DEGREES = 2.0;
    public static final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    public static final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    public static final double YAW_PID_P = 0.005;
    public static final double YAW_PID_I = 0.0;
    public static final double YAW_PID_D = 0.0;
    // --------------------------
    //  Vision
    // --------------------------
    // Vuforia Vision
    public static final String webcamName = "Webcam";
    public static final String VUFORIA_KEY =
            "AWWCp8z/////AAABmQqV/K50N0OTqlyIYanMsyQ6huM5ckTKtdjF0/gyTwTINZPIGhLWxx3ag5PUmAw90BOHnZh3arwMSH0sjWZUM7wTJG/rcPmsj3MFp2eSPPc+osid/6jBjyg8YuhBYFN8jO3YvFlo/24qqX8K1DWOX8GU7dAfZEIhI71HCmY+pRWIGxKWyXxkpf3xULPPommaHqF7wSA/z37uQs+zSTs9SJKxiGvUlF7oYkVkURIuzovMKiK7rRqQT/dmCKH/JFpxgl8Er3O50/DL03EMmmNbjkiqA4vAU7wwD8rTkHympjAl7MnSmQRtXWxyRUildftpaQr7rD8vuz+4A6j/+/nKeTUanIi1fPMuE0Xa+Cth7SDr";
    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch;
    // Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;
    // Center-Support Targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;
    public static final float bridgeRotZ = 180;
    // Perimeter Targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;
    // Camera
    public static final float phoneXRotate = 0;
    public static final float phoneYRotate = -90;
    public static final float phoneZRotate = 0;
    public static final float CAMERA_FORWARD_DISPLACEMENT  = 0 * mmPerInch;
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
    public static final float CAMERA_LEFT_DISPLACEMENT = 0 * mmPerInch;
    public static final OpenGLMatrix robotFromCamera = MathFx.createMatrix(CAMERA_FORWARD_DISPLACEMENT,
            CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, phoneXRotate, phoneYRotate,
            phoneZRotate);
    //  TF Vision
    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    // --------------------------
    //  Wheel-Based Odometry
    // --------------------------
    public static final double ENCODER_DIFFERENCE = 1;
    public static final double HORIZONTAL_OFFSET = 0;
    public static final int dt = 100;
    public static final String horizontal = "Horizontal";

}
