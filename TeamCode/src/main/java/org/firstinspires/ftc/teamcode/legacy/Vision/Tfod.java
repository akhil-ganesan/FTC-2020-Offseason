package org.firstinspires.ftc.teamcode.legacy.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;

import java.util.List;

@Deprecated
public class Tfod extends Vision {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    List<Recognition> updatedRecognitions;

    @Override
    public void init(HardwareMap ahMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        //parameters.cameraDirection = Constants.CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = ahMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.TFOD_MODEL_ASSET, Constants.LABEL_FIRST_ELEMENT, Constants.LABEL_SECOND_ELEMENT);

        tfod.activate();
        updatedRecognitions = tfod.getUpdatedRecognitions();
    }

    /*@Override
    public StateMachine getStateMachine() {
        return null;
    }

     */

    @Override
    public boolean search(GameElement gameElement) {
        return false;
    }

    public Float[] SkySearch() {
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == "Skystone") {
                    return new Float[]{recognition.getTop(), recognition.getLeft(),
                            recognition.getBottom(), recognition.getRight()};
                }
            }
        }
        return null;
    }

}
