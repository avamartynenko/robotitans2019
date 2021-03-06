package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftMech {

    DcMotor liftElevator;
    Servo grabber;
    Servo twister;
    HardwareMap hwMap =  null;
    public final int UP = 1;
    public final int DOWN = 2;
    public double UP_SPEED = 0.0;
    public double DOWN_SPEED = 0.5;
    public double GRABBER_LOCK_POSITION = 0.95; //
    public double GRABBER_RELEASE_POSITION = 0.4; // 0.2
    public double TWISTER_HOME_POSITION = 1.0; //
    public double TWISTER_DELIVER_POSITION = 0.23;



    public LiftMech(HardwareMap parentHwMap){

        liftElevator = parentHwMap.get(DcMotor.class, "liftElevator");
        grabber = parentHwMap.get(Servo.class, "grabber");
        twister= parentHwMap.get(Servo.class, "twister");
        grabber.setPosition(GRABBER_RELEASE_POSITION);
        twister.setPosition(TWISTER_HOME_POSITION);
        //grabber.setDirection(DcMotorSimple.Direction.REVERSE);
        //twister.setDirection(DcMotorSimple.Direction.FORWARD);


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

        if(speed >= 0) {
            liftElevator.setPower(speed);
        } else {
            liftElevator.setPower(DOWN_SPEED);
        }

    }



    public void grabStone(double speed){

        //grabber.setPower(speed);
    }



    public void twistLift(double speed){

        //twister.setPower(speed);
    }











}


