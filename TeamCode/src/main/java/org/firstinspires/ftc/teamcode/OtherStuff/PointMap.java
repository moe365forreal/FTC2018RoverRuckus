package org.firstinspires.ftc.teamcode.OtherStuff;

import java.util.ArrayList;

/**
 * Created by rohankanchana on 9/15/18.
 */

//0, 0 is the blue square, 72, 72 is the red square.
public class PointMap {
    public static int[] getMark(String name) {
        if (name.toLowerCase().equals("mars")) {
            return vuforiaTags.mars;
        } else if (name.toLowerCase().equals("galaxy")) {
            return vuforiaTags.galaxy;
        } else if (name.toLowerCase().equals("rover")) {
            return vuforiaTags.rover;
        } else {
            return vuforiaTags.moon;
        }
    }

    public static class redDepot {
        public static int[] bottomLeft = new int[]{0, 0};
        public static int[] topLeft = new int[]{0, 12};
        public static int[] topRight = new int[]{12, 12};
        public static int[] bottomRight = new int[]{12, 0};
        public static int[] middle = new int[]{6, 6};
    }

    public static class blueDepot {
        public static int[] bottomLeft = new int[]{60, 60};
        public static int[] topLeft = new int[]{60, 72};
        public static int[] topRight = new int[]{72, 72};
        public static int[] bottomRight = new int[]{72, 60};
        public static int[] middle = new int[]{66, 66};
        public static int[] bottomMiddle = new int[]{66, 60};
    }

    public static int[] center = new int[]{36, 36};

    public static class blueSamplingDepot {
        public static int[] left = new int[]{24, 13};
        public static int[] middle = new int[]{18, 18};
        public static int[] right = new int[]{13, 24};
    }

    public static class blueSamplingCrater {
        public static int[] left = new int[]{13, 49};
        public static int[] middle = new int[]{18, 54};
        public static int[] right = new int[]{24, 60};
    }

    public static class redSamplingDepot {
        public static int[] left = new int[]{49, 60};
        public static int[] middle = new int[]{54, 54};
        public static int[] right = new int[]{60, 49};
    }

    public static class redSamplingCrater {
        public static int[] left = new int[]{60, 24};
        public static int[] middle = new int[]{54, 18};
        public static int[] right = new int[]{49, 13};
    }

    public static class corners {
        public static int[] bottomLeft = new int[]{0, 0};
        public static int[] topLeft = new int[]{0, 72};
        public static int[] topRight = new int[]{72, 72};
        public static int[] bottomRight = new int[]{72, 0};
    }

    public static class vuforiaTags {
        public static int[] mars = new int[]{36, 72}; //front, audience facing wall
        public static int[] rover = new int[]{72, 36};//blue wall
        public static int[] moon = new int[]{0, 36};//red wall
        public static int[] galaxy = new int[]{36, 0};//back wall
    }

    public static class blueCrater {
        public static int[] one = new int[]{72, 24};
        public static int[] two = new int[]{60, 20};
        public static int[] three = new int[]{50, 12};
        public static int[] four = new int[]{46, 0};
    }

    public static class redCrater {
        public static int[] one = new int[]{0, 48};
        public static int[] two = new int[]{12, 50};
        public static int[] three = new int[]{22, 60};
        public static int[] four = new int[]{26, 72};
    }

    public static class landingSquare {
        public static int[] blue = new int[]{54, 36};
        public static int[] front = new int[]{36, 54};
        public static int[] back = new int[]{36, 20};
        public static int[] red = new int[]{20, 36};
    }
}
