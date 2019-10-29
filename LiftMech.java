package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftMech {

    DcMotor liftElevator;
    CRServo grabber;
    CRServo twister;
    HardwareMap hwMap =  null;
    public final int UP = 1;
    public final int DOWN = 2;
    public double STOP_SPEED = 0.0;


    public LiftMech(HardwareMap parentHwMap){

        liftElevator = parentHwMap.get(DcMotor.class, "liftElevator");
        grabber = parentHwMap.get(CRServo.class, "grabber");
        twister= parentHwMap.get(CRServo.class, "twister");


    }

    private void setDirection(int direction){


        switch (direction) {
            case UP:
                liftElevator.setDirection(DcMotor.Direction.FORWARD);
                break;

            case DOWN:
                liftElevator.setDirection(DcMotor.Direction.REVERSE);
                break;
        }

    }


    public void runElevator(double speed){

        liftElevator.setPower(speed);
    }



    public void grabStone(double speed){

        grabber.setPower(speed);
    }



    public void twistLift(double speed){

        twister.setPower(speed);
    }











}


