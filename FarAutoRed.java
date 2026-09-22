package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "FarAutoRed (Blocks to Java)", preselectTeleOp = "RedFieldCentricIndexVelocitysensorlights")
public class FarAutoRed extends LinearOpMode {

  private DcMotor leftFront;
  private DcMotor rightFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  private DcMotor shooterRight;
  private DcMotor shooterLeft;
  private DcMotor feeder;
  private DcMotor intake;
  private CRServo turret;
  private Servo spoonM;
  private Servo spoonL;
  private Servo spoonR;
  private ColorSensor colorR1_REV_ColorRangeSensor;
  private ColorSensor colorR2_REV_ColorRangeSensor;
  private Servo RGBR;
  private Servo Hood;
  private Servo RGBM;
  private Servo RGBL;
  private ColorSensor colorL1_REV_ColorRangeSensor;
  private ColorSensor colorL2_REV_ColorRangeSensor;
  private ColorSensor colorM1_REV_ColorRangeSensor;
  private ColorSensor colorM2_REV_ColorRangeSensor;
  private TouchSensor counterclockwiseLimit;
  private TouchSensor clockwiseLimit;

  double p;
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  int farVelocity;
  int tagname;
  double tagPosition;

  /**
   * Describe this function...
   */
  private void timedStrafe(int strafe, double timeSeconds, double Speed) {
    leftFront.setPower(strafe * -1 * Speed);
    rightFront.setPower(strafe * 1 * Speed);
    leftBack.setPower(strafe * 1 * Speed);
    rightBack.setPower(strafe * -1 * Speed);
    sleep((long) (timeSeconds * 1000));
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
    shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
    feeder = hardwareMap.get(DcMotor.class, "feeder");
    intake = hardwareMap.get(DcMotor.class, "intake");
    turret = hardwareMap.get(CRServo.class, "turret");
    spoonM = hardwareMap.get(Servo.class, "spoonM");
    spoonL = hardwareMap.get(Servo.class, "spoonL");
    spoonR = hardwareMap.get(Servo.class, "spoonR");
    colorR1_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorR1");
    colorR2_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorR2");
    RGBR = hardwareMap.get(Servo.class, "RGBR");
    Hood = hardwareMap.get(Servo.class, "Hood");
    RGBM = hardwareMap.get(Servo.class, "RGBM");
    RGBL = hardwareMap.get(Servo.class, "RGBL");
    colorL1_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorL1");
    colorL2_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorL2");
    colorM1_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorM1");
    colorM2_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorM2");
    counterclockwiseLimit = hardwareMap.get(TouchSensor.class, "counterclockwiseLimit");
    clockwiseLimit = hardwareMap.get(TouchSensor.class, "clockwiseLimit");

    USE_WEBCAM = true;
    initAprilTag();
    // Put initialization blocks here.
    farVelocity = 1480;
    leftFront.setDirection(DcMotor.Direction.REVERSE);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    shooterRight.setDirection(DcMotor.Direction.FORWARD);
    shooterLeft.setDirection(DcMotor.Direction.REVERSE);
    feeder.setDirection(DcMotor.Direction.FORWARD);
    intake.setDirection(DcMotor.Direction.FORWARD);
    shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      p = 0.11;
      while (opModeIsActive() && Math.abs(p) > 0.1) {
        turretTrack();
        telemetry.update();
      }
      turret.setPower(0);
      telemetry.addData("p", p);
      telemetry.update();
      sleep((long) (1.5 * 1000));
      telemetry.update();
      farShot();
      sleep((long) (1.5 * 1000));
      sleep((long) (0.5 * 1000));
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
      timedStrafe(-5, 0.85, 0.3);
      intake.setPower(-1);
      shooterLeft.setPower(0);
      shooterRight.setPower(0);
      feeder.setPower(0);
      timedDrive(-0.4, 0, 2.4);
      timedDrive(0.6, 0, 1.2);
      timedStrafe(1, 0.8, 1);
      intake.setPower(0);
      p = 0.11;
      while (opModeIsActive() && Math.abs(p) > 0.08) {
        turretTrack();
        telemetry.update();
      }
      turret.setPower(0);
      farShot();
      sleep((long) (1.5 * 1000));
      sleep((long) (0.5 * 1000));
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
      sleep((long) (1.5 * 1000));
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
      intake.setPower(0);
      telemetry.addData("p", 2);
      timedDrive(-0.5, 0, 0.7);
      telemetry.addData("key", 123);
      telemetry.update();
      telemetry.addData("key", 456);
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void timedDrive(double forward, int turn, double timeSeconds) {
    leftFront.setPower(forward + turn);
    rightFront.setPower(forward - turn);
    leftBack.setPower(forward + turn);
    rightBack.setPower(forward - turn);
    sleep((long) (timeSeconds * 1000));
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void timedLaunch(double launchSpeed) {
    shooterRight.setPower(launchSpeed);
    shooterLeft.setPower(launchSpeed);
    sleep((long) (3.5 * 1000));
    // amount of pause for re-spin up
    // Number of times repeated feeder delay
    timed_feed(1, 0.1, 1.5, 4);
    shooterRight.setPower(0);
    shooterLeft.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void timed_feed(int intakeSpeed, double timeSeconds, double pauseSeconds, int repeat) {
    // Timed Delay Repeat For Feeding balls.
    for (int count = 0; count < repeat; count++) {
      timedDrive(null, null, null);
      intake.setPower(intakeSpeed);
      sleep((long) (timeSeconds * 1000));
      intake.setPower(0);
      sleep((long) (pauseSeconds * 1000));
    }
  }

  /**
   * Describe this function...
   */
  private void rightBucket() {
    if (((OpticalDistanceSensor) colorR1_REV_ColorRangeSensor).getLightDetected() + ((OpticalDistanceSensor) colorR2_REV_ColorRangeSensor).getLightDetected() < 0.3) {
      RGBR.setPosition(0);
    } else if (colorR1_REV_ColorRangeSensor.green() > colorR1_REV_ColorRangeSensor.blue()) {
      RGBR.setPosition(0.5);
    } else if (colorR1_REV_ColorRangeSensor.green() < colorR1_REV_ColorRangeSensor.blue()) {
      RGBR.setPosition(0.722);
    } else if (colorR2_REV_ColorRangeSensor.green() > colorR2_REV_ColorRangeSensor.blue()) {
      RGBR.setPosition(0.5);
    } else if (colorR2_REV_ColorRangeSensor.green() < colorR2_REV_ColorRangeSensor.blue()) {
      RGBR.setPosition(0.722);
    }
  }

  /**
   * Describe this function...
   */
  private void farShot() {
    Hood.setPosition(0.44);
    feeder.setPower(1);
    ((DcMotorEx) shooterLeft).setVelocity(farVelocity);
    ((DcMotorEx) shooterRight).setVelocity(farVelocity);
  }

  /**
   * Describe this function...
   */
  private void hood(double hoodPosition) {
    Hood.setPosition(hoodPosition);
  }

  /**
   * Describe this function...
   */
  private void activateSpoons() {
    if (RGBM.getPosition() == 0 && RGBL.getPosition() == 0 && RGBR.getPosition() == 0) {
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
    } else if (RGBM.getPosition() == 0 && RGBL.getPosition() == 0) {
      spoonR.setPosition(1);
      sleep((long) (0.25 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
    } else if (RGBM.getPosition() == 0 && RGBR.getPosition() == 0) {
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep(1 * 1000);
    } else if (RGBL.getPosition() == 0 && RGBR.getPosition() == 0) {
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep(1 * 1000);
    } else if (RGBM.getPosition() == 0) {
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.25 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
    } else if (RGBR.getPosition() == 0) {
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep(1 * 1000);
    } else if (RGBL.getPosition() == 0) {
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.25 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
    } else {
      spoonM.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(1);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0);
      sleep(1 * 1000);
    }
  }

  /**
   * Describe this function...
   */
  private void leftBucket() {
    if (((OpticalDistanceSensor) colorL1_REV_ColorRangeSensor).getLightDetected() + ((OpticalDistanceSensor) colorL2_REV_ColorRangeSensor).getLightDetected() < 0.3) {
      RGBL.setPosition(0);
    } else if (colorL1_REV_ColorRangeSensor.green() > colorL1_REV_ColorRangeSensor.blue()) {
      RGBL.setPosition(0.5);
    } else if (colorL1_REV_ColorRangeSensor.green() < colorL1_REV_ColorRangeSensor.blue()) {
      RGBL.setPosition(0.722);
    } else if (colorL2_REV_ColorRangeSensor.green() > colorL2_REV_ColorRangeSensor.blue()) {
      RGBL.setPosition(0.5);
    } else if (colorL2_REV_ColorRangeSensor.green() < colorL2_REV_ColorRangeSensor.blue()) {
      RGBL.setPosition(0.722);
    }
  }

  /**
   * Describe this function...
   */
  private void middleBucket() {
    if (((OpticalDistanceSensor) colorM1_REV_ColorRangeSensor).getLightDetected() + ((OpticalDistanceSensor) colorM2_REV_ColorRangeSensor).getLightDetected() < 0.3) {
      RGBM.setPosition(0);
    } else if (colorM1_REV_ColorRangeSensor.green() > colorM1_REV_ColorRangeSensor.blue()) {
      RGBM.setPosition(0.5);
    } else if (colorM1_REV_ColorRangeSensor.green() < colorM1_REV_ColorRangeSensor.blue()) {
      RGBM.setPosition(0.722);
    } else if (colorM2_REV_ColorRangeSensor.green() > colorM2_REV_ColorRangeSensor.blue()) {
      RGBM.setPosition(0.5);
    } else if (colorM2_REV_ColorRangeSensor.green() < colorM2_REV_ColorRangeSensor.blue()) {
      RGBM.setPosition(0.722);
    }
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    int targdetect;
    AprilTagDetection myAprilTagDetection;
    double targdist;

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    targdetect = JavaUtil.listLength(myAprilTagDetections);
    telemetry.addData("# AprilTags Detected", targdetect);
    telemetry.addLine("");
    myAprilTagDetection = (((AprilTagDetection) JavaUtil.inListGet(myAprilTagDetections, JavaUtil.AtMode.RANDOM, (int) 0, false)));
    telemetry.addData("apriltag data", "");
    if (myAprilTagDetections.isEmpty()) {
      telemetry.addData("key", 123);
      tagPosition = null;
      tagname = null;
    } else {
      tagPosition = myAprilTagDetection.ftcPose.x;
      tagname = myAprilTagDetection.id;
      targdist = myAprilTagDetection.ftcPose.y;
      telemetry.addData("apriltag data", JavaUtil.formatNumber(targdist, 6, 1));
    }
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Create an AprilTagProcessor by calling build.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Describe this function...
   */
  private void turretTrack() {
    double cameraOffset;
    double kp;
    double error;

    telemetryAprilTag();
    if (tagname == 24) {
      // Run auto code Here
      if (tagPosition != null) {
        cameraOffset = 1.5;
        kp = 0.025;
        error = tagPosition - cameraOffset;
        p = 1 * error * kp;
        telemetry.addData("p", tagname);
        telemetry.update();
      }
    } else {
      turret.setPower(0);
    }
    if (counterclockwiseLimit.isPressed()) {
      p = 1;
    } else if (clockwiseLimit.isPressed()) {
      p = -1;
    }
    turret.setPower(p);
  }
}
