package org.firstinspires.ftc.teamcode.lib.util;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MathFx {

    public static double scale(double lower, double upper, double value) {
        return Math.max(lower, Math.min(value, upper));
    }

    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    public static OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public static String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

}
