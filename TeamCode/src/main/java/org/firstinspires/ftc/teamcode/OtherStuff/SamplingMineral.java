package org.firstinspires.ftc.teamcode.OtherStuff;

public class SamplingMineral {
    String label;
    int xpos;
    String loc;
    double estimatedAngle;

    public double getEstimatedAngle() {
        return estimatedAngle;
    }

    public void setEstimatedAngle(double estimatedAngle) {
        this.estimatedAngle = estimatedAngle;
    }

    public SamplingMineral(String label, int xpos, String loc, double ea) {
        this.label = label;
        this.xpos = xpos;
        this.loc = loc;
        this.estimatedAngle = ea;
    }

    public String getLoc() {
        return loc;
    }

    public void setLoc(String loc) {
        this.loc = loc;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public int getXpos() {
        return xpos;
    }

    public void setXpos(int xpos) {
        this.xpos = xpos;
    }
}
