package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMech {

    DcMotor intakeLeft;
    DcMotor intakeRight;
    HardwareMap hwMap =  null;
    public final int OUT = 1;
    public final int IN = 2;
    public double INTAKE_SPEED = 0.3;
    public double STOP_SPEED = 0.0;


    public IntakeMech(HardwareMap parentHwMap){

        intakeLeft = parentHwMap.get(DcMotor.class, "intakeLeft");
        intakeRight = parentHwMap.get(DcMotor.class, "intakeRight");
        setDirection(IN);

    }

    private void setDirection(int direction){


        switch (direction) {
            case OUT:
                intakeLeft.setDirection(DcMotor.Direction.FORWARD);
                intakeRight.setDirection(DcMotor.Direction.FORWARD);
                break;

            case IN:
                intakeLeft.setDirection(DcMotor.Direction.REVERSE);
                intakeRight.setDirection(DcMotor.Direction.REVERSE);
                break;
        }

    }



    public void run(double speed){

        //clockwise 1.0 = 100% power to 1.5 = 0% power
        //counterclockwise 2.0 = 100% to 1.5 0% power

        intakeLeft.setPower(speed);
        intakeRight.setPower(speed);
    }


    public void run(){

        run(INTAKE_SPEED);

    }


    public void stop(){

        run(STOP_SPEED);
    }





}


