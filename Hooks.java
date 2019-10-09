package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hooks extends LinearOpMode {

    CRServo hookRight;
    CRServo hookLeft;
    HardwareMap hwMap =  null;
    public final int DOWN = 1;
    public final int UP = 2;
    public double LATCH_SPEED = 0.1;
    public double STOP_SPEED = 0.0;

    @Override
    public void runOpMode() {

    }

    public Hooks(HardwareMap parentHwMap){

        hookLeft = parentHwMap.get(CRServo.class, "hookLeft");
        hookRight = parentHwMap.get(CRServo.class, "hookRight");



    }
    private void setDirection(int direction){


        switch (direction) {
            case UP:
                hookLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                hookRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case DOWN:
                hookLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                hookRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
        }

    }





    public void run(double speed){

        //clockwise 1.0 = 100% power to 1.5 = 0% power
        //counterclockwise 2.0 = 100% to 1.5 0% power

        hookLeft.setPower(LATCH_SPEED);
        hookRight.setPower(LATCH_SPEED);
    }

    public void latch(){

        run(0);
        setDirection(DOWN);
        run(LATCH_SPEED);

    }

    public void release(){

        run(0);
        setDirection(UP);
        run(LATCH_SPEED);
    }

    public void completeRelease(){

        run(0);
    }





}


