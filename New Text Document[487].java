
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Autonomous Blue Position", group="Pushbot")

public class Autonomous_Blue extends LinearOpMode {


  private DcMotorEx MotorRight;
  private DcMotorEx MotorLeft;
  private DcMotorEx frontRightWheel;
  private DcMotorEx frontLeftMotor; 
  private DcMotorEx Arm;
  private DcMotorEx ArmJoint;
  private CRServo Claw;
  private DcMotorEx TopWheel;
  
  static double PI=3.141592;
  static double CIRCUMFERENCE = 76/25.4 * PI;
  static double GEAR_3_RATIO = 2.89;
  static double GEAR_4_RATIO = 3.61;
  static double GEAR_5_RATIO = 5.23;
  static double COUNTS_PER_IN_DRIVE = 28*GEAR_3_RATIO*GEAR_4_RATIO*GEAR_3_RATIO/CIRCUMFERENCE;


  static double COUNT_PER_DEGREE_ARM   = 28*GEAR_3_RATIO*GEAR_4_RATIO*GEAR_5_RATIO*(125/30)*(90/45)/360;
  static double COUNT_PER_DEGREE_ARMJOINT   = 28*GEAR_3_RATIO * GEAR_4_RATIO * GEAR_5_RATIO/360;

  static double COUNT_PER_360_ROTATE=6400;
  static double COUNT_PER_360_ROTATE_SPEED   = 25.5;

  // Function to Move in straight line
 
 // Distance in inches
 // Speed in inches/sec
  public void move(double distance, double speed, String status)
  {
    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    MotorRight.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));  
    MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorRight.setVelocity(  COUNTS_PER_IN_DRIVE * speed ); // Set Velocity is in Ticks per Second
  
    frontLeftMotor.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeftMotor.setVelocity( COUNTS_PER_IN_DRIVE * speed);
    
    frontRightWheel.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
    frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRightWheel.setVelocity(  COUNTS_PER_IN_DRIVE * speed);
    
    
    MotorLeft.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
    MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    MotorLeft.setVelocity( COUNTS_PER_IN_DRIVE * speed );
    
    
    while (opModeIsActive() && 
    (MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()) ){
      telemetry.addData("Status", status);
      telemetry.addData("Motor Left", MotorLeft.getCurrentPosition() - MotorLeft.getTargetPosition());
      telemetry.addData("Motor Right", MotorRight.getCurrentPosition() - MotorRight.getTargetPosition());
      telemetry.addData("Motor Front Right", frontRightWheel.getCurrentPosition() - frontRightWheel.getTargetPosition());
      telemetry.addData("Motor Front Left", frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition());
      telemetry.update();
    }
  }
  
  // Rotate around center of Robot
  // Degrees and degrees per sec
  public void rotate(double angle, double speed, String status)
  {
  
    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
      MotorRight.setTargetPosition( (int)((angle/360)*COUNT_PER_360_ROTATE));
      MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      MotorRight.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );
      
      MotorLeft.setTargetPosition( (int)((-angle/360)*COUNT_PER_360_ROTATE));
      MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      MotorLeft.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );
    
      frontRightWheel.setTargetPosition( (int)((angle/360)*COUNT_PER_360_ROTATE));
      frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontRightWheel.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );
      
      frontLeftMotor.setTargetPosition( (int)((-angle/360)*COUNT_PER_360_ROTATE));
      frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeftMotor.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );



    while (opModeIsActive() && 
     (MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()) ){
      telemetry.addData("Status", status);
      telemetry.addData("Diff", frontRightWheel.getCurrentPosition() - frontRightWheel.getTargetPosition());
      telemetry.update();
      
    }
    
  }
  
    public void strafe(double distance, double angle, double speed, String status)
  {
    
    
    MotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    double y = Math.cos(angle*PI/180)*speed;
    double x = Math.sin(angle*PI/180)*speed;


    MotorRight.setVelocity( (y + x)*COUNTS_PER_IN_DRIVE ); // Set Velocity is in Ticks per Secon
    frontLeftMotor.setVelocity( (y + x)*COUNTS_PER_IN_DRIVE);
    frontRightWheel.setVelocity((y-x)*COUNTS_PER_IN_DRIVE );
    MotorLeft.setVelocity( (y-x)*COUNTS_PER_IN_DRIVE);
    
    telemetry.addData("Status", status);
    telemetry.addData("Time to move", (distance/speed));
    telemetry.update();
    
    sleep((long)(distance/speed)*1000);
    
    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
  }
  
  
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    MotorLeft = hardwareMap.get(DcMotorEx.class, "MotorLeft");
    MotorRight = hardwareMap.get(DcMotorEx.class, "MotorRight");
    frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
    frontRightWheel = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
    ArmJoint = hardwareMap.get(DcMotorEx.class, "arm joint");
    Arm = hardwareMap.get(DcMotorEx.class, "Arm");
    TopWheel = hardwareMap.get(DcMotorEx.class, "topwheelmotor");
    Claw = hardwareMap.get(CRServo.class, "claw");
    
      // Reverse one of the drive motors.
    frontRightWheel.setDirection(DcMotorEx.Direction.REVERSE);
    ArmJoint.setDirection(DcMotorEx.Direction.REVERSE);

    
    MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    waitForStart();
    

    if (opModeIsActive()) {
      
    
      // Grab Initial block
      Claw.setPower(-.2);sleep(1200);
      Arm.setTargetPosition((int)(40* COUNT_PER_DEGREE_ARM));
      Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Arm.setVelocity(60*COUNT_PER_DEGREE_ARM );
     /*
      while (opModeIsActive() && Arm.isBusy() ){
        telemetry.addData("Arm Be Move", Arm.getVelocity());
        telemetry.update();
      }
    */
      ArmJoint.setTargetPosition((int)(50* COUNT_PER_DEGREE_ARMJOINT));
      ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ArmJoint.setVelocity(45*COUNT_PER_DEGREE_ARMJOINT);
     
     // If we want to wait until the armjoint is done.. uncomment this
     /*
      while (opModeIsActive() && ArmJoint.isBusy() ){
        telemetry.addData("Arm Joint Be Move", ArmJoint.getVelocity());
        telemetry.update();
      }
      */
      
      //CHECK DISTANCES
      move(10, 30, "Robot go forward"); 
      strafe(18, 90, 5,"");
      move(12,15,"robot go forward");
      
      //Drop the block on the top level
      Claw.setPower(1); sleep(500);
      Claw.setPower(0);
      
      // To Roll a Duck.. Uncomment this
      
      move(-8, 20, "roboreverse");
      rotate(180, 45, "TARGET LOCKED");
      strafe (60,-90,5,"");
      move(8, 6, "roboreverse");
      TopWheel.setPower (.5);sleep(2500);
      move(-4, 20, "roboreverse");
      rotate(90, 45, "TARGET LOCKED"); //sleep(2000);
      move(100,50,"Robot go BRRRRRRRRR");
      
      /*
      // Move back to Warehouse
      move(-18, 25, "roboreverse");
      rotate(90, 45, "TARGET LOCKED"); //sleep(2000);
      strafe (8,-90,10,"Strafe");
      move(58,25,"Robot go BRRRRRRRRR");
     */
    }
  }
}

