package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Thread.sleep;

public class Arm {



    CRServo dropServo;
    CRServo collectServo;
    public int name;  // Front or Back
    HardwareMap hwMap =  null;
    public final int DOWN = 1;
    public final int UP = 2;
    public static final int FRONT_ARM = 3;
    public static final int BACK_ARM = 4;
    public double LATCH_SPEED = 0.1;
    public double STOP_SPEED = 1.5;


    public Arm(HardwareMap parentHwMap, int nameOfArm){
        this.name = nameOfArm;

        dropServo = parentHwMap.get(CRServo.class, "dropServo" + nameOfArm);
        collectServo = parentHwMap.get(CRServo.class, "collectServo" + nameOfArm);


    }

    private void setDirectionDropServo(int direction, CRServo servoType){

        if((name == FRONT_ARM && direction == DOWN) || (name == BACK_ARM && direction == UP) ){

            servoType.setDirection(DcMotorSimple.Direction.REVERSE);

        } else if((name == FRONT_ARM && direction == UP) || (name == BACK_ARM && direction == DOWN)){

            servoType.setDirection(DcMotorSimple.Direction.FORWARD);

        }
    }


    public void test(double speed){
        collectServo.setPower(speed);

    }





    public void liftUp(double power){

        dropServo.setPower(0);
        setDirectionDropServo(UP, dropServo);
        dropServo.setPower(power);


    }

    public void moveToHomePosition(){

        liftUp(0.1);
        releaseStone(0.1);

    }

    public void goDown(double power){

        dropServo.setPower(0);
        setDirectionDropServo(DOWN, dropServo);
        dropServo.setPower(power);

    }

    public void latchStone(double power){

        collectServo.setPower(0);
        setDirectionDropServo(DOWN, collectServo);
        collectServo.setPower(power);
    }

    public void releaseStone(double power){

        collectServo.setPower(0);
        setDirectionDropServo(UP, collectServo);
        collectServo.setPower(power);

    }


    public void stop(){

        dropServo.setPower(0);
        collectServo.setPower(0);
    }

}


