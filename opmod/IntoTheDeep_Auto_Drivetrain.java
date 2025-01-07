package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntoTheDeep_Auto_Drivetrain {
    HardwareMap hardwareMap;
    Telemetry telemetry;
//
    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor Fright;
    private DcMotor Bright;

    private DcMotor Pick;

    private DcMotor Drop;


// Servos
    private Servo hand;
    private  Servo claw;
    private Servo RoClaw;
    private Servo SpecClaw;

    private Servo arm;

    private Servo outake;



    //Calculation for motor to calculate inches
    static final double     COUNTS_PER_MOTOR_REV_5202    = 751.8 ;    // eg: go bilda 5202 Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV_5203    = 537.7 ;    // eg: go bilda 5203 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES_GB_MW   = 4.03 ;     // For figuring circumference
    static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_5203         = (COUNTS_PER_MOTOR_REV_5203 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_GB_MW * 3.1415);
    static final double     COUNTS_PER_INCH_5202         = (COUNTS_PER_MOTOR_REV_5202 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    public IntoTheDeep_Auto_Drivetrain(HardwareMap hMap, Telemetry tel) {

        hardwareMap = hMap;
        telemetry = tel;

    }
    public void configDriveTrain()
    {
        // group DriveTranin
        FLeft = hardwareMap.dcMotor.get("FLeft");
        BLeft = hardwareMap.dcMotor.get("BLeft");
        Fright = hardwareMap.dcMotor.get("FRight");
        Bright = hardwareMap.dcMotor.get("BRight");
        hand=hardwareMap.get(Servo.class,"hand");
        claw=hardwareMap.get(Servo.class,"claw");
        RoClaw=hardwareMap.get(Servo.class,"RoClaw");
        SpecClaw=hardwareMap.get(Servo.class,"SpecClaw");
        arm=hardwareMap.get(Servo.class,"arm");
        outake=hardwareMap.get(Servo.class,"outake");
        Pick = hardwareMap.dcMotor.get("pick");
        Drop = hardwareMap.dcMotor.get("drop");

//DriveTrain Motor directions
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake and outake motor
        Pick.setDirection(DcMotorSimple.Direction.FORWARD);
        Drop.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo positions
        RoClaw.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.REVERSE);
        hand.setDirection(Servo.Direction.FORWARD);
        SpecClaw.setDirection(Servo.Direction.FORWARD);
        outake.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.FORWARD);

        //Encoder STOPING and RESETING
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Encoders for Drop and Pick slides
        Drop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//Running DriveTrain Encoders
        FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Running Drop and pick encoders
        Pick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

  private void setSlidesEncoder(double inch){
        double tar = inch*COUNTS_PER_INCH_5202;

  }

    private void setDriveTrainEncoder(double inch){
        double targetCount = inch*COUNTS_PER_INCH_5203;
        int targetCountFL = FLeft.getCurrentPosition()+(int)targetCount;
        int targetCountFR = Fright.getCurrentPosition()+(int)targetCount;
        int targetCountBL = BLeft.getCurrentPosition()+(int)targetCount;
        int targetCountBR = Bright.getCurrentPosition()+(int)targetCount;
        double pwr=1;

        FLeft.setTargetPosition(targetCountFL);
        Fright.setTargetPosition(targetCountFR);
        BLeft.setTargetPosition(targetCountBL);
        Bright.setTargetPosition(targetCountBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void stopDriveTrain(){
        FLeft.setPower(0);
        Fright.setPower(0);
        Bright.setPower(0);
        BLeft.setPower(0);
    }

    public void moveForwardAndBackword(double inch){

        setDriveTrainEncoder(inch);
        while (FLeft.isBusy() || Fright.isBusy() || BLeft.isBusy() || Bright.isBusy()) {
            FLeft.setPower(.4);
            Fright.setPower(.4);
            Bright.setPower(.4);
            BLeft.setPower(.4);
        }
       stopDriveTrain();

    }
    public void moveLeftRight(double inch) {

        double targetCount = inch*COUNTS_PER_INCH_5203;
        int targetCountFL = FLeft.getCurrentPosition()-(int)targetCount;
        int targetCountFR = Fright.getCurrentPosition()+(int)targetCount;
        int targetCountBL = BLeft.getCurrentPosition()-(int)targetCount;
        int targetCountBR = Bright.getCurrentPosition()+(int)targetCount;
        double pwr=.4;
        if(inch<0){
            pwr= pwr *(-1);
        }

        FLeft.setTargetPosition(targetCountFL);
        Fright.setTargetPosition(targetCountFR);
        BLeft.setTargetPosition(targetCountBL);
        Bright.setTargetPosition(targetCountBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FLeft.isBusy() || Fright.isBusy() || BLeft.isBusy() || Bright.isBusy()) {
          FLeft.setPower(- pwr);
          Fright.setPower(pwr);
          BLeft.setPower(-pwr);
          Bright.setPower(pwr);

      }
      stopDriveTrain();
    }
    public void SlideLeftAndRight(double inch) {

        double targetCount = inch*COUNTS_PER_INCH_5203;
        int targetCountFL = FLeft.getCurrentPosition()+(int)targetCount;
        int targetCountFR = Fright.getCurrentPosition()-(int)targetCount;
        int targetCountBL = BLeft.getCurrentPosition()-(int)targetCount;
        int targetCountBR = Bright.getCurrentPosition()+(int)targetCount;
        double pwr=.4;
        if(inch<0){
            pwr= pwr *(-1);
        }

        FLeft.setTargetPosition(targetCountFL);
        Fright.setTargetPosition(targetCountFR);
        BLeft.setTargetPosition(targetCountBL);
        Bright.setTargetPosition(targetCountBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FLeft.isBusy() || Fright.isBusy() || BLeft.isBusy() || Bright.isBusy()) {
            FLeft.setPower(- pwr);
            Fright.setPower(pwr);
            BLeft.setPower(-pwr);
            Bright.setPower(pwr);

        }
        stopDriveTrain();



    }



}//class
