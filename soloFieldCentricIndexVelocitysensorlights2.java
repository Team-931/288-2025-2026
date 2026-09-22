package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "soloFieldCentricIndexVelocitysensorlights2 (Blocks to Java)")
public class soloFieldCentricIndexVelocitysensorlights2 extends LinearOpMode {

  private IMU imu_IMU;
  private DcMotor rightFront;
  private DcMotor leftFront;
  private DcMotor leftBack;
  private DcMotor rightBack;
  private DcMotor shooterRight;
  private DcMotor shooterLeft;
  private DcMotor intake;
  private Servo RGBM;
  private Servo RGBL;
  private Servo RGBR;
  private Servo spoonM;
  private Servo spoonL;
  private Servo spoonR;
  private Servo Hood;
  private DcMotor feeder;
  private ColorSensor colorR1_REV_ColorRangeSensor;
  private ColorSensor colorR2_REV_ColorRangeSensor;
  private ColorSensor colorL1_REV_ColorRangeSensor;
  private ColorSensor colorL2_REV_ColorRangeSensor;
  private ColorSensor colorM1_REV_ColorRangeSensor;
  private ColorSensor colorM2_REV_ColorRangeSensor;

  double multiplier;
  YawPitchRollAngles _7ByawPitchRollAnglesVariable_7D;
  double rotX;
  float x;
  float rx;
  double rotY;
  int closeVelocity;
  int farVelocity;
  float y;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int maxVelocity;
    double driveDeadZone;
    float raw;

    imu_IMU = hardwareMap.get(IMU.class, "imu");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
    shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
    intake = hardwareMap.get(DcMotor.class, "intake");
    RGBM = hardwareMap.get(Servo.class, "RGBM");
    RGBL = hardwareMap.get(Servo.class, "RGBL");
    RGBR = hardwareMap.get(Servo.class, "RGBR");
    spoonM = hardwareMap.get(Servo.class, "spoonM");
    spoonL = hardwareMap.get(Servo.class, "spoonL");
    spoonR = hardwareMap.get(Servo.class, "spoonR");
    Hood = hardwareMap.get(Servo.class, "Hood");
    feeder = hardwareMap.get(DcMotor.class, "feeder");
    colorR1_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorR1");
    colorR2_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorR2");
    colorL1_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorL1");
    colorL2_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorL2");
    colorM1_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorM1");
    colorM2_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorM2");

    // Initializes the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics
    // Control Hub or Expansion Hub, specifying the hub's arbitrary orientation on
    // the robot via an Orientation block that describes the rotation that would
    // need to be applied in order to rotate the hub from having its logo facing up
    // and the USB ports facing forward, to its actual orientation on the robot.
    imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(90, 0, 0))));
    // Reverseing drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftFront.setDirection(DcMotor.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    shooterRight.setDirection(DcMotor.Direction.FORWARD);
    shooterLeft.setDirection(DcMotor.Direction.REVERSE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    farVelocity = 1470;
    maxVelocity = 2200;
    closeVelocity = 1225;
    driveDeadZone = 0.05;
    if (Math.abs(gamepad1.left_stick_y) < driveDeadZone) {
      raw = gamepad1.left_stick_y;
    }
    if (Math.abs(gamepad1.left_stick_x) < driveDeadZone) {
      raw = gamepad1.left_stick_x;
    }
    if (Math.abs(gamepad1.right_stick_x) < driveDeadZone) {
      raw = gamepad1.right_stick_x;
    }
    waitForStart();
    if (opModeIsActive()) {
      imu_IMU.resetYaw();
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        _7ByawPitchRollAnglesVariable_7D = imu_IMU.getRobotYawPitchRollAngles();
        if (gamepad1.x) {
          multiplier = 0.5;
        } else {
          multiplier = 1;
        }
        gamepad();
        setPower();
        // Pressing Y will Reset the IMU for Field Centric
        if (gamepad1.y) {
          imu_IMU.resetYaw();
        }
        if (gamepad1.left_bumper) {
          intake.setPower(1);
        } else if (gamepad1.left_bumper_was_released()) {
          intake.setPower(0);
        }
        if (gamepad1.right_bumper_was_pressed()) {
          intake.setPower(-1);
        } else if (gamepad1.right_bumper_was_released()) {
          intake.setPower(0);
        }
        hoodMovement();
        setFlywheelVelocity();
        rightBucket();
        leftBucket();
        middleBucket();
        data();
        spoonPreCycle();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void activateSpoons() {
    if (RGBM.getPosition() == 0 && RGBL.getPosition() == 0 && RGBR.getPosition() == 0) {
      spoonM.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(0.985);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(0.32);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0.005);
      sleep(1 * 1000);
    } else if (RGBM.getPosition() == 0 && RGBL.getPosition() == 0) {
      spoonR.setPosition(0.32);
      sleep((long) (0.25 * 1000));
      spoonR.setPosition(0.005);
      sleep(1 * 1000);
    } else if (RGBM.getPosition() == 0 && RGBR.getPosition() == 0) {
      spoonL.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep(1 * 1000);
    } else if (RGBL.getPosition() == 0 && RGBR.getPosition() == 0) {
      spoonM.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(0.985);
      sleep(1 * 1000);
    } else if (RGBM.getPosition() == 0) {
      spoonL.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(0.32);
      sleep((long) (0.25 * 1000));
      spoonR.setPosition(0.005);
      sleep(1 * 1000);
    } else if (RGBR.getPosition() == 0) {
      spoonM.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(0.985);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep(1 * 1000);
    } else if (RGBL.getPosition() == 0) {
      spoonM.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(0.985);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(0.32);
      sleep((long) (0.25 * 1000));
      spoonR.setPosition(0.005);
      sleep(1 * 1000);
    } else {
      spoonM.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(0.985);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(0.32);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0.005);
      sleep(1 * 1000);
    }
  }

  /**
   * Describe this function...
   */
  private void gamepad() {
    double theta;

    // Use left stick to drive and right stick to turn
    // You may have to negate the sticks. When you
    // negate a stick, negate all other instances of the stick
    x = gamepad1.left_stick_x;
    y = -gamepad1.left_stick_y;
    rx = gamepad1.right_stick_x;
    theta = -_7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES);
    // Calculated Values
    rotX = x * Math.cos(theta / 180 * Math.PI) - y * Math.sin(theta / 180 * Math.PI);
    rotY = x * Math.sin(theta / 180 * Math.PI) + y * Math.cos(theta / 180 * Math.PI);
  }

  /**
   * Describe this function...
   */
  private void setPower() {
    // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
    // We negate this value so that the topmost position corresponds to maximum forward power.
    leftFront.setPower((rotY + rotX + rx) * multiplier);
    rightFront.setPower(((rotY - rotX) - rx) * multiplier);
    // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
    // We negate this value so that the topmost position corresponds to maximum forward power.
    leftBack.setPower(((rotY - rotX) + rx) * multiplier);
    rightBack.setPower(((rotY + rotX) - rx) * multiplier);
  }

  /**
   * Describe this function...
   */
  private void data() {
    YawPitchRollAngles robotOrientation;

    robotOrientation = imu_IMU.getRobotYawPitchRollAngles();
    telemetry.addData("Yaw - rotation about Robot Z", robotOrientation.getYaw(AngleUnit.DEGREES));
    telemetry.addData("Pitch - rotation about Robot X", robotOrientation.getPitch(AngleUnit.DEGREES));
    telemetry.addData("Roll - rotation about Robot Y", robotOrientation.getRoll(AngleUnit.DEGREES));
    telemetry.addData("Shoot Left Velocity", ((DcMotorEx) shooterLeft).getVelocity());
    telemetry.addData("Shoot Right Velocity", ((DcMotorEx) shooterRight).getVelocity());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void closeShot() {
    Hood.setPosition(0.3);
    feeder.setPower(1);
    ((DcMotorEx) shooterLeft).setVelocity(closeVelocity);
    ((DcMotorEx) shooterRight).setVelocity(closeVelocity);
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
  private void hoodMovement() {
    if (gamepad2.dpad_left_was_pressed()) {
      Hood.setPosition(0);
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
  private void setFlywheelVelocity() {
    if (gamepad1.start) {
      shooterLeft.setPower(-1);
      shooterRight.setPower(-1);
    } else if (gamepad1.a) {
      farShot();
      if (((DcMotorEx) shooterLeft).getVelocity() >= farVelocity) {
        activateSpoons();
      }
    } else if (false) {
      closeShot();
      if (((DcMotorEx) shooterLeft).getVelocity() >= closeVelocity) {
        activateSpoons();
      }
    } else {
      feeder.setPower(0);
      shooterLeft.setPower(0);
      shooterRight.setPower(0);
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
   * Describe this function...
   */
  private void spoonPreCycle() {
    if (gamepad1.back) {
      spoonM.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonM.setPosition(0.985);
      sleep((long) (0.22 * 1000));
      spoonL.setPosition(0.68);
      sleep((long) (0.25 * 1000));
      spoonL.setPosition(1);
      sleep((long) (0.22 * 1000));
      spoonR.setPosition(0.32);
      sleep((long) (0.4 * 1000));
      spoonR.setPosition(0.005);
      sleep(1 * 1000);
    }
  }
}
