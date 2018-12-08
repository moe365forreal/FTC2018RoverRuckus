package org.firstinspires.ftc.teamcode.OtherStuff;

import org.firstinspires.ftc.teamcode.OtherStuff.Pair;

import java.util.ArrayList;

public class Pathing {
    public ArrayList<Pair> pathing;
    public ArrayList<Pair> extra;
    public String error;

    public Pathing(ArrayList<Pair> pathing, String error) {
        this.pathing = pathing;
        this.error = error;
        this.extra = null;
    }

    public Pathing(ArrayList<Pair> pathing, ArrayList<Pair> extra, String error) {
        this.pathing = pathing;
        this.extra = extra;
        this.error = error;
    }
}
