package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Thread.sleep;

public class Arm {



    CRServo dropServo;
    CRServo collectServo;
    public String name;  // Front or Back
    HardwareMap hwMap =  null;
    public final int DOWN = 1;
    public final int UP = 2;
    public double LATCH_SPEED = 0.1;
    public double STOP_SPEED = 1.5;


    public Arm(HardwareMap parentHwMap, String nameOfArm){
        this.name = nameOfArm;

        dropServo = parentHwMap.get(CRServo.class, "dropServo" + nameOfArm);
        collectServo = parentHwMap.get(CRServo.class, "collectServo" + nameOfArm);


    }

    /*private void setDirection(int direction){

        if (direction < 0) {
            direction = this.UP;
        }
        else {
            direction = this.DOWN;
        }

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

*/



    public void test(double speed){

        collectServo.setPower(speed);
    }




    public void liftUp(){


    }

    public void goDown(){


    }

    public void latchStone(){


    }

    public void releaseStone(){


    }

}


