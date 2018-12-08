package org.firstinspires.ftc.teamcode;

/**
 * Created by rohankanchana on 9/25/18.
 */

public class Methods {
    public static double closestAngleDifference(double ang1, double ang2) {
        double difference = Math.abs(ang2 - ang1);
        double secondDifference = 360 - difference;
        double returnVal = difference < secondDifference ? difference : secondDifference;

//        if (difference > 180) {
//            returnVal = -returnVal;
//        }
//
        double actualDiff = ang2 - ang1;
        if (-180 < actualDiff && actualDiff < 0) {
            returnVal = -returnVal;
        } else if (actualDiff > 180) {
            returnVal = -returnVal;
        }

        return returnVal;
    }

    public static double scaleToRange(double number, double numberMin, double numberMax, double newMin, double newMax) {
        return ((number - numberMin) / (numberMax - numberMin)) * (newMax - newMin) + newMin;
    }
}
