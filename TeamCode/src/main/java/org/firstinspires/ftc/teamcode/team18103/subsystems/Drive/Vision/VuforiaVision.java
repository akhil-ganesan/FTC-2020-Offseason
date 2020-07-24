package org.firstinspires.ftc.teamcode.team18103.subsystems.Drive.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.bridgeRotY;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.bridgeRotZ;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.bridgeX;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.bridgeY;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.bridgeZ;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.halfField;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.quadField;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.stoneZ;

public class VuforiaVision extends Subsystem {

    private VuforiaLocalizer vuforia;
    private WebcamName webcamName;

    private OpenGLMatrix lastKnownLocation = MathFx.createMatrix(0, 0, 0, 0, 0, 0);

    private float X = 0;
    private float Y = 0;
    private float Theta = 0;

    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void init(HardwareMap ahMap) {

        webcamName = ahMap.get(WebcamName.class, Constants.webcamName);

        int cameraMonitorViewId = ahMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", ahMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = true;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initVuforia();

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(Constants.robotFromCamera, parameters.cameraDirection);
        }
    }

    public boolean search(VuforiaTrackable target) {
        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
            return true;
        } else {
            return false;
        }
    }

    public void getRobotLocation() {
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (search(trackable)) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }
                VectorF translation = lastKnownLocation.getTranslation();
                X = translation.get(0)/Constants.mmPerInch;
                Y = translation.get(1)/Constants.mmPerInch;
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastKnownLocation, EXTRINSIC, XYZ, DEGREES);
                Theta = rotation.thirdAngle;
            }
            break;
        }
    }

    private void initVuforia() {
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables.addAll(targetsSkyStone);
        // Setting Traceable Target Locations
        stoneTarget.setLocation(MathFx.createMatrix(0, 0, stoneZ, 90, 0, -90));

        blueFrontBridge.setLocation(MathFx.createMatrix(-bridgeX, bridgeY, bridgeZ, 0, bridgeRotY, bridgeRotZ));

        blueRearBridge.setLocation(MathFx.createMatrix(-bridgeX, bridgeY, bridgeZ, 0, -bridgeRotY, bridgeRotZ));

        redFrontBridge.setLocation(MathFx.createMatrix(-bridgeX, -bridgeY, bridgeZ, 0, -bridgeRotY, 0));

        redRearBridge.setLocation(MathFx.createMatrix(bridgeX, -bridgeY, bridgeZ, 0, bridgeRotY, 0));

        red1.setLocation(MathFx.createMatrix(quadField, -halfField, mmTargetHeight, 90, 0, 180));

        red2.setLocation(MathFx.createMatrix(-quadField, -halfField, mmTargetHeight, 90, 0, 180));

        front1.setLocation(MathFx.createMatrix(-halfField, -quadField, mmTargetHeight, 90, 0, 90));

        front2.setLocation(MathFx.createMatrix(-halfField, quadField, mmTargetHeight, 90, 0, 90));

        blue1.setLocation(MathFx.createMatrix(-quadField, halfField, mmTargetHeight, 90, 0, 0));

        blue2.setLocation(MathFx.createMatrix(quadField, halfField, mmTargetHeight, 90, 0, 0));

        rear1.setLocation(MathFx.createMatrix(halfField, quadField, mmTargetHeight, 90, 0, -90));

        rear2.setLocation(MathFx.createMatrix(halfField, -quadField, mmTargetHeight, 90, 0, -90));
    }

    public float getX() {
        getRobotLocation();
        return X;
    }

    public float getY() {
        getRobotLocation();
        return Y;
    }

    public float getTheta() {
        getRobotLocation();
        return Theta;
    }

}
