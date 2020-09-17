package org.firstinspires.ftc.teamcode.lib.util;

public class MeanOptimizedDataFusionModel {

    double bias = 0;

    public MeanOptimizedDataFusionModel() {

    }

    public MeanOptimizedDataFusionModel(double bias) {
        setBias(bias);
    }

    public double fuse(double[] data) {
        double sum = 0;
        for (double i : data) {
            sum += i;
        }
        return (sum / (double)(data.length)) + bias;
    }

    public double fuse(double[] data, double[] weights) {
        double sum = 0;
        double len = 0;
        for (int i = 0; i < data.length; i++) {
            sum += data[i] * weights[i];
            len += weights[i];
        }
        return (sum / len) + bias;
    }

    public void setBias(double bias) {
        this.bias = bias;
    }

}
