package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    double leftPow, rightPow; //using the tilerunner, so only two sides are distinguished in powering
    double drive, turn;

    DcMotor flyWheel, intake, wobbleLift;
    DcMotor rightFront, rightBack, leftFront, leftBack;

    Servo transfer1;
    Servo transfer2;
    Servo wobbleGrab;
    Servo moveToShoot;
    Servo storageArm;

    boolean shootToggle, grabToggle, transferToggle, moveToggle, storageToggle; //one button two actions -> a monostable circuit
    boolean transitDown, wobbleOpen, cycling, fedLaunch, ringPushed;
    boolean pad1DriveToggle, pad1DualStickDrive; //monostable circuit for switching between dual stick drive and right stick drive
    boolean pad2DriveToggle, pad2DualStickDrive;//same as above but for pad 2
    double transPosUp, transPosDown;           //transfer mechanism servos

    public Robot(){

    }
}
