package org.firstinspires.ftc.teamcode.lib.concept;

public class StatesConcept {
    public static State state = State.OFF;

    public enum State {
        OFF(0d, 0d, 0d, 0d);

        final double p1;
        final double p2;
        final double p3;
        final double p4;

        State(double p1, double p2, double p3, double p4) {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
            this.p4 = p4;
        }
        /*
        final double[] power = new double[]{p1, p2, p3, p4};

        public double[] getPower() {
            return power;
        }

         */



    }
}
