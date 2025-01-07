package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonBlue_Basket extends LinearOpMode {
    IntoTheDeep_Auto_Drivetrain DriveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain = new IntoTheDeep_Auto_Drivetrain(hardwareMap,telemetry);
        DriveTrain.configDriveTrain();
       waitForStart();
       // DriveTrain.moveForwardAndBackword(-5);
       // sleep(2000);
       // DriveTrain.moveForwardAndBackword(5);
       // sleep(2000);
      //  DriveTrain.moveLeftRight(20);
      //  sleep(2000);
        DriveTrain.SlideLeftAndRight(20);//slide to the left
        sleep(100);
        DriveTrain.SlideLeftAndRight(-20);//slide to the right




    }
}
