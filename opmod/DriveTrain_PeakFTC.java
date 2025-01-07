package org.firstinspires.ftc.teamcode;
import static android.text.Selection.moveDown;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// into the deep code working with gobilda linear slide
@TeleOp
public class DriveTrain_PeakFTC extends OpMode {

    // Declare motor variables
    private DcMotor frontLeftMotor;// port number 0 rev expansion hub
    private DcMotor frontRightMotor;// port number 1 control hub
    private DcMotor rearLeftMotor;//port number 2expansion hub
    private DcMotor rearRightMotor;// port number 3 control hub
    private DcMotor hanging;// port number 1 expansion hub
    private DcMotor arm;// port number 2 expansion hub

    private DcMotor LinPick;
    //Intake things
    private Servo claw;// EX port 4
    private Servo hand; // Ex port 5
    private Servo dropper;


    private Servo dropperHelp;
    //private Servo SPC; // Specimen holder mover
   // private Servo SPH; // specimen holder/dropper
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 752 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private Thread dropTheSample;
    private Thread dipositTheSample;
    private int level =0;
    private boolean isTrigger=false;

    // Initialize hardware
    @Override
    public void init() {
        // Map motors to hardware configuration names
        frontLeftMotor = hardwareMap.dcMotor.get("Fleft");
        frontRightMotor = hardwareMap.dcMotor.get("Fright");
        rearLeftMotor = hardwareMap.dcMotor.get("Rleft");
        rearRightMotor = hardwareMap.dcMotor.get("Rright");
        hanging = hardwareMap.dcMotor.get("hanging");
        arm = hardwareMap.dcMotor.get("arm");
        LinPick=hardwareMap.dcMotor.get("LinPick");
        hand=hardwareMap.get(Servo.class,"hand");
        claw=hardwareMap.get(Servo.class,"claw");
        dropper = hardwareMap.get(Servo.class,"dropper");
        dropperHelp = hardwareMap.get(Servo.class,"dropper2");
       // SPC = hardwareMap.get(Servo.class,"spc");
        //SPH = hardwareMap.get(Servo.class,"sph");

        // Set motor directions (adjust based on your robot)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        dropper.setDirection(Servo.Direction.FORWARD);
        dropperHelp.setDirection(Servo.Direction.REVERSE);
        hand.setDirection(Servo.Direction.REVERSE);

        hanging.setDirection(DcMotor.Direction.FORWARD);// change direction if does not mach with controler controls
        arm.setDirection(DcMotor.Direction.FORWARD);
        LinPick.setDirection((DcMotorSimple.Direction.FORWARD));
        //SPH.setDirection(Servo.Direction.REVERSE);

        // reset all the encoder before run
        hanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //IN TAKE MOTORS
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinPick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DRIVETRAIN MOTORS
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor modes (RUN_USING_ENCODER for better control)

        //DRIVETRAIN MOTORS
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //INTAKE MOTORS
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinPick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //OUTAKE LIN
        hanging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //INTAKE/OUTAKE SERVOS
        hand.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        dropTheSample = new Thread(() -> {
            while(true) {
                if (level == 1) {
                    dropTheSampleAtLevelOne();
                    level =0;
                } else if (level == 2) {
                    dropTheSampleAtLevelTwo();
                    level =0;
                }
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
               // telemetry.addData("Thread call", " level %d",level);
               // telemetry.update();
            }
        });

        dipositTheSample = new Thread(() -> {
            while(true) {
               if(isTrigger){
                   dropTheSampleintoLinearSideBucket();
                   isTrigger=false;
               }
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                // telemetry.addData("Thread call", " level %d",level);
                // telemetry.update();
            }
        });
    }

    private void resetEncoder(DcMotor motor){
        if(motor!=null){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);      //start again
        }
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        telemetry.addData("Linear slid Currently at", " at %7d",
                hanging.getCurrentPosition());

        telemetry.addData("Rleft Currently at", " at %7d",
                rearLeftMotor.getCurrentPosition());
        telemetry.addData("Fleft Currently at", " at %7d",
                frontLeftMotor .getCurrentPosition());
        telemetry.addData("RRight Currently at", " at %7d",
                rearRightMotor.getCurrentPosition());
        telemetry.addData("FRight Currently at", " at %7d",
                rearLeftMotor.getCurrentPosition());


        telemetry.addData("ARM Currently at", " at %7d",
                arm.getCurrentPosition());

        telemetry.update();
    }


    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
        resetEncoder(frontLeftMotor);
        resetEncoder(frontRightMotor);
        resetEncoder(rearLeftMotor);
        resetEncoder(rearRightMotor);
        resetEncoder(arm);
        resetEncoder(hanging);
        armATRest();
        dropTheSample.start();
        dipositTheSample.start();
    }
	
    // Run the robot
    @Override
    public void loop() {

        // Get gamepad inputs for driving
        double drive = gamepad1.right_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double rearLeftPower = 0;
        double rearRightPower = 0;


        double PWR=  LimitPower(gamepad2.right_stick_y);

        arm.setPower(PWR);
        LinPick.setPower(gamepad2.left_stick_y);



        // Calculate motor powers based on Mecanum drive equations
        /* frontLeftPower = drive - strafe - rotate;
         frontRightPower = drive + strafe + rotate;
         rearLeftPower = drive + strafe -rotate;
         rearRightPower = drive - strafe + rotate;*/

        frontLeftPower = drive - strafe - rotate;
        frontRightPower = drive + strafe + rotate;
        rearLeftPower = drive + strafe - rotate;
        rearRightPower = drive - strafe + rotate;

      //  if(gamepad1.right_bumper){
         //   SPC.setPosition(1);// close
        //}
        //if(gamepad1.left_bumper){
        //    SPC.setPosition(0); //open
       // }
       if(gamepad1.right_trigger>0){

           linPickMovedForward(1035);
       }
       // if(gamepad1.left_trigger>0){
        //}
        if(gamepad1.dpad_up){
			level=2;
        }
        if(gamepad1.dpad_down){
			level=1;
        }
        if(gamepad1.dpad_left) {
            isTrigger=true;
        }
        if(gamepad1.x) {
            //pickTheSample();
            claw.setPosition(0.3);
            runtime.reset();
            while ( runtime.seconds()< 0.5) ;
            hand.setPosition(0.85);
        }
	    if(gamepad1.y){
            claw.setPosition(1);
            runtime.reset();
            while (runtime.seconds()<0.5);
            hand.setPosition(0);

	    }
        if(gamepad1.left_bumper){
            runtime.reset();
            while (runtime.seconds()<5){
                arm.setPower(1);
                LinPick.setPower(-1);
            }
        }


        // Set motor powers (scale to keep values between -0.9 and 0.9)
        if(strafe > 0){ // left  move
            frontLeftMotor.setPower(scaleInput(frontLeftPower));
            frontRightMotor.setPower(scaleInput(frontRightPower));
            rearLeftMotor.setPower((scaleInput(rearLeftPower)-0.09));
            rearRightMotor.setPower((scaleInput(rearRightPower)+0.1));
        }else if(strafe  < 0) { // right move
            frontLeftMotor.setPower(scaleInput(frontLeftPower));
            frontRightMotor.setPower(scaleInput(frontRightPower));
            rearLeftMotor.setPower((scaleInput(rearLeftPower)+0.1));
            rearRightMotor.setPower((scaleInput(rearRightPower)-0.1));

        }else if(strafe==0 && (drive!=0 ||rotate!=0)){ // forward/backward/rotate
            frontLeftMotor.setPower(scaleInput(frontLeftPower));
            frontRightMotor.setPower(scaleInput(frontRightPower));
            rearLeftMotor.setPower(scaleInput(rearLeftPower));
            rearRightMotor.setPower(scaleInput(rearRightPower));
        }else { // no move
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
        }
        telemetry.addData("Linear slid dropper Currently at", " at %7d",
                hanging.getCurrentPosition());

        telemetry.addData("Rleft Currently at", " at %7d",
                rearLeftMotor.getCurrentPosition());
        telemetry.addData("Fleft Currently at", " at %7d",
                frontLeftMotor .getCurrentPosition());
        telemetry.addData("RRight Currently at", " at %7d",
                rearRightMotor.getCurrentPosition());
        telemetry.addData("FRight Currently at", " at %7d",
                rearLeftMotor.getCurrentPosition());


        telemetry.addData("ARM Currently at", " at %7d",
                arm.getCurrentPosition());

        telemetry.addData("LinPick Currently at", " at %7d",
                LinPick.getCurrentPosition());

        telemetry.update();
    }
    private void pickTheSample() {

    }
    private double LimitPower(double rightStickY) {
        return Math.max(-0.5, Math.min(0.5, rightStickY));
    }

    private void linPickMovedForward(int encoderVal)
    {
        double currentPos = LinPick.getCurrentPosition();
        double targetPod = currentPos+encoderVal;
        double timeOutSec = 1.75;
        double pwr=0.75;
        runtime.reset();
        // 1500 each slides
        while ( (currentPos < encoderVal) && (runtime.seconds()< timeOutSec) ) {
            currentPos = LinPick.getCurrentPosition();
            telemetry.addData("Arm Currently at", " at %7d",
                    LinPick.getCurrentPosition());
            telemetry.update();

            LinPick.setPower(pwr);
        }
        LinPick.setPower(0);
    }
    private void linPickMovedReverse(int encoderVal)
    {
        double currentPos = LinPick.getCurrentPosition();
        double timeOutSec =0.7;
        double pwr=-0.75; // to move reverse
        runtime.reset();
        // 1500 each slides
        while ( (runtime.seconds()< timeOutSec) ) {
            currentPos = LinPick.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    LinPick.getCurrentPosition());
            telemetry.update();

            LinPick.setPower(pwr);
        }
        LinPick.setPower(0);
    }

    private void dropTheSampleAtLevelOne() {
        double currentPos = hanging.getCurrentPosition();
        double timeOutSec = 6;
        double pwr=0;
        runtime.reset();
        // 1500 each slides
        while ( (currentPos < 1500) && (runtime.seconds()< timeOutSec) ) {
            currentPos = hanging.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    hanging.getCurrentPosition());
            telemetry.update();

            if(runtime.seconds()<3)
                pwr = 1;
            else if (runtime.seconds()>3 && runtime.seconds()<4)
                pwr = 0.2;
            else if (runtime.seconds()>4)
                pwr = 0.1;

            hanging.setPower(pwr);
        }
        dropper.setPosition(1);
        dropperHelp.setPosition(1);
        runtime.reset();
        while(runtime.seconds()<4 ){
            hanging.setPower(0.01);
        }
        dropper.setPosition(0.25);
        dropperHelp.setPosition(0.25);
        runtime.reset();
        while(runtime.seconds()<1 ){
            telemetry.addData("Currently at", " at %7d",
                    hanging.getCurrentPosition());
            telemetry.update();
        }
        currentPos = hanging.getCurrentPosition();

        timeOutSec = 3;
        while (( currentPos >50 ) && (runtime.seconds()< timeOutSec) ) {
            currentPos = hanging.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    hanging.getCurrentPosition());
            telemetry.update();

            pwr = -1;

            hanging.setPower(pwr);
        }
        hanging.setPower(0);
    }

    private double scaleInput(double input) {
        return Math.max(-0.9, Math.min(0.9, input));
    }
    private void dropTheSampleintoLinearSideBucket() {
        // how much Arm should swing to pi2ck and drop the samples
        linPickMovedReverse(0); // get back to rest position
        int armSwingEncoderVal =850;
        runtime.reset();
        int currentPos = arm.getCurrentPosition();
        int targetPos = currentPos+1228;
        arm.setTargetPosition( targetPos);
        double pwr =0.4;
        double timeOutSec = 1;
        runtime.reset();
        // 1500 each slides  -- 3750
        //while ( (currentPos < 850) && (runtime.seconds()< timeOutSec) ) {
        while ( (currentPos < targetPos) && (runtime.seconds()< timeOutSec) ) {
            currentPos = arm.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    arm.getCurrentPosition());
            telemetry.update();

            if (runtime.seconds() < 2)
                pwr = 0.5;
            else if (runtime.seconds() > 3 && runtime.seconds() < 4)
                pwr = 0.2;
            else if (runtime.seconds() > 4)
                pwr = 0.1;

            arm.setPower(pwr);
        }
        arm.setPower(0);
        runtime.reset();
        claw.setPosition(0.3);
        while(runtime.seconds()<1);

        runtime.reset();
        currentPos = arm.getCurrentPosition();
        targetPos = currentPos-1350;
        arm.setTargetPosition(targetPos);
        while ( (currentPos >targetPos) && (runtime.seconds()< timeOutSec) ) {
            currentPos = arm.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    arm.getCurrentPosition());
            telemetry.update();

            if (runtime.seconds() < 2)
                pwr = -1;
            else if (runtime.seconds() > 3 && runtime.seconds() < 4)
                pwr = -0.2;
            else if (runtime.seconds() > 4)
                pwr = -0.1;

            arm.setPower(pwr);
        }
        arm.setPower(0);

    }
    private void armATRest(){
        double currentPos = arm.getCurrentPosition();
        double pwr = 0;
        double timeOutSec =2;
        runtime.reset();
        while ( (currentPos >-500) && (runtime.seconds()< timeOutSec) ) {
            currentPos = arm.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    arm.getCurrentPosition());
            telemetry.update();

            if (runtime.seconds() < 2)
                pwr = -1;
            else if (runtime.seconds() > 3 && runtime.seconds() < 4)
                pwr = -0.2;
            else if (runtime.seconds() > 4)
                pwr = -0.1;

            arm.setPower(pwr);
        }
        arm.setPower(0);
    }
    private void dropTheSampleAtLevelTwo() {
        double currentPos = hanging.getCurrentPosition();
        double pwr =0.4;
        double timeOutSec = 6;
        runtime.reset();
        // 1500 each slides  -- 3750
        while ( (currentPos < 4000) && (runtime.seconds()< timeOutSec) ) {
            currentPos = hanging.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    hanging.getCurrentPosition());
            telemetry.update();

            if(runtime.seconds()<3)
                pwr = 1;
            else if (runtime.seconds()>3 && runtime.seconds()<4)
                pwr = 0.2;
            else if (runtime.seconds()>4)
                pwr = 0.1;

            hanging.setPower(pwr);
        }
        dropper.setPosition(1);
        dropperHelp.setPosition(1);
        runtime.reset();
        while(runtime.seconds()<4 ){
            hanging.setPower(0.01);
        }
        dropper.setPosition(0.25);
        dropperHelp.setPosition(0.25);
        runtime.reset();
        while(runtime.seconds()<1 ){
            telemetry.addData("Currently at", " at %7d",
                    hanging.getCurrentPosition());
            telemetry.update();
        }
         currentPos = hanging.getCurrentPosition();

         timeOutSec = 3;
        while (( currentPos >50 ) && (runtime.seconds()< timeOutSec) ) {
            currentPos = hanging.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    hanging.getCurrentPosition());
            telemetry.update();

            pwr = -1;

            hanging.setPower(pwr);
        }
        hanging.setPower(0);
    }
    public void moveRight(){
        // Get gamepad inputs for driving
        double drive = 0;
        double strafe = 1;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;
        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }
    public void moveLeft(){
        // Get gamepad inputs for driving
        double drive = 0;
        double strafe = -1;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }
    public void moveForward(){
        // Get gamepad inputs for driving
        double drive = -1;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }

    public void moveBackward(){
        // Get gamepad inputs for driving
        double drive = 1;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }

    public void moveStop() {
        // Get gamepad inputs for driving
        double drive = 0;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor

    }
    private void setMovePower(double fLeftPower,double fRightPower, double rLeftPower,double rRightPower )   {
        // Set motor powers (scale to keep values between -1 and 1)
        frontLeftMotor.setPower(scaleInput(fLeftPower));
        frontRightMotor.setPower(scaleInput(fRightPower));
        rearLeftMotor.setPower(scaleInput(rLeftPower));
        rearRightMotor.setPower(scaleInput(rRightPower));
    }
}