package org.firstinspires.ftc.teamcode.OtherStuff;

public class RobotMovement {
    public static final double VERTICAL_HORIZONTAL_INCHES = 1;
    public static final double DIAGONAL_45DEGREES_INCHES = 1.41421;
    public int direction;
    public double inches;

    public class Directions {
        public static final int N = 0;
        public static final int S = 4;
        public static final int E = 2;
        public static final int W = 6;
        public static final int NE = 1;
        public static final int SE = 3;
        public static final int NW = 7;
        public static final int SW = 5;
    }

    public static int rotate(int degrees, int direction) {
        int change = degrees / 45;
        return ((direction + change) % 8);
    }

    public RobotMovement(int d, double i) {
        this.direction = d;
        this.inches = i;
    }
}
