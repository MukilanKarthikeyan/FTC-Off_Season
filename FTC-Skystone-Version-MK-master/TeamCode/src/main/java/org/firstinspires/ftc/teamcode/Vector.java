package org.firstinspires.ftc.teamcode;
import java.util.*;

public class Vector{
    private double x, y;
    private double theta, dist;
    private double[] polarVec;
    private double[] cartesianVec;

    public Vector(boolean PolarOrCartesian, double AngOrX, double distOrY){
        polarVec = new double[2];
        cartesianVec = new double[2];
        if(PolarOrCartesian){
            this.theta = AngOrX;
            this.dist = distOrY;
            this.x = Math.cos(theta)*distOrY;
            this.y = Math.sin(theta)*distOrY;
            polarVec[0] = this.dist;
            polarVec[1] = this.theta;
            cartesianVec[0] = this.x;
            cartesianVec[1] = this.y;
        }
        else{
            this.theta = Math.atan2(distOrY/AngOrX);
            this.dist = Math.sqrt((AngOrX**2)+(distOrY**2));
            this.x = AngOrX;
            this.y = distOrY;
            polarVec[0] = this.dist;
            polarVec[1] = this.theta;
            cartesianVec[0] = this.x;
            cartesianVec[1] = this.y;
        }
    }

    public void setVecotrCar(double x, double y){
        this.x = x;
        this.y = y;
    }
    public void setVectorPolar(double ang, double dist){
        this.theta = ang;
        this.dist = dist;
    }
    public double getAngle(){
        return theta;
    }

    public double getMagnitude(){
        return dist;
    }
    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }
    public
    public String toStringPolar(){
        return("Polar: ("+Double.toString(this.theta)+","+Double.toString(this.dist)+")");
    }
    public String toStringCart(){
        return("Cartesian: ("+Double.toString(this.x)+","+Double.toString(this.y)+")");
    }

}