package frc.robot.subsystems.user_input;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.BandedJoystick;
import frc.robot.util.Smoother;

/**
 * Handles input from 2 joysticks and an xbox controller.
 *
 * @author Elan Ronen
 * @author Ian Keller
 * @see <a
 *     href="https://www.digitalcombatsimulator.com/upload/iblock/3c2/Template-T16000m-Hotas.jpg">Joystick
 *     Button Map</a>
 */
public class UserInput extends SubsystemBase implements RobotMap {

  private static UserInput instance;

  public static UserInput getInstance() {
    return instance == null ? instance = new UserInput() : instance;
  }

  private final BandedJoystick leftJoystick;
  private final BandedJoystick rightJoystick;
  private final Smoother.Wrapper leftXInput;
  private final Smoother.Wrapper leftYInput;
  private final Smoother.Wrapper rightXInput;
  private final XboxController operatorController;

  private UserInput() {

    leftJoystick = new BandedJoystick(LEFT_JOYSTICK, 0.1, 0.1);
    rightJoystick = new BandedJoystick(RIGHT_JOYSTICK, 0.1, 0.1);
    leftXInput = Smoother.wrap(2, leftJoystick::getXBanded);
    leftYInput = Smoother.wrap(2, leftJoystick::getYBanded);
    rightXInput = Smoother.wrap(1, rightJoystick::getXBanded);
    operatorController = new XboxController(2);
  }

  public void update() {
    leftXInput.update();
    leftYInput.update();
    rightXInput.update();
  }

  // LEFT JOYSTICK

  public double leftJoystickX() {
    return leftXInput.get();
  }

  public double leftJoystickY() {
    return leftYInput.get();
  }

  public boolean leftJoystickButton(int button) {
    return leftJoystick.getRawButton(button);
  }

  public boolean leftJoystickButtonPressed(int button) {
    return leftJoystick.getRawButtonPressed(button);
  }

  public boolean leftJoystickButtonReleased(int button) {
    return leftJoystick.getRawButtonReleased(button);
  }

  public int leftJoystickPOV() {
    return leftJoystick.getPOV();
  }

  public boolean leftJoystickTrigger() {
    return leftJoystick.getTrigger();
  }

  public boolean leftJoystickTriggerPressed() {
    return leftJoystick.getTriggerPressed();
  }

  public boolean leftJoystickTriggerReleased() {
    return leftJoystick.getTriggerReleased();
  }

  // RIGHT JOYSTICK

  public double rightJoystickX() {
    return rightXInput.get();
  }

  public boolean rightJoystickButton(int button) {
    return rightJoystick.getRawButton(button);
  }

  public boolean rightJoystickButtonPressed(int button) {
    return rightJoystick.getRawButtonPressed(button);
  }

  public boolean rightJoystickButtonReleased(int button) {
    return rightJoystick.getRawButtonReleased(button);
  }

  public int rightJoystickPOV() {
    return rightJoystick.getPOV();
  }

  public boolean rightJoystickTrigger() {
    return rightJoystick.getTrigger();
  }

  public boolean rightJoystickTriggerPressed() {
    return rightJoystick.getTriggerPressed();
  }

  public boolean rightJoystickTriggerReleased() {
    return rightJoystick.getTriggerReleased();
  }

  // OPERATOR CONTROLLER

  public double controllerLeftX() {
    return operatorController.getLeftX();
  }

  public double controllerLeftY() {
    return operatorController.getLeftY();
  }

  public double controllerRightX() {
    return operatorController.getRightX();
  }

  public double controllerRightY() {
    return operatorController.getRightY();
  }

  public boolean controllerAButton() {
    return operatorController.getAButton();
  }

  public boolean controllerAButtonPressed() {
    return operatorController.getAButtonPressed();
  }

  public boolean controllerAButtonReleased() {
    return operatorController.getAButtonReleased();
  }

  public boolean controllerBButton() {
    return operatorController.getBButton();
  }

  public boolean controllerBButtonPressed() {
    return operatorController.getBButtonPressed();
  }

  public boolean controllerBButtonReleased() {
    return operatorController.getBButtonReleased();
  }

  public boolean controllerXButton() {
    return operatorController.getXButton();
  }

  public boolean controllerXButtonPressed() {
    return operatorController.getXButtonPressed();
  }

  public boolean controllerXButtonReleased() {
    return operatorController.getXButtonReleased();
  }

  public boolean controllerYButton() {
    return operatorController.getYButton();
  }

  public boolean controllerYButtonPressed() {
    return operatorController.getYButtonPressed();
  }

  public boolean controllerYButtonReleased() {
    return operatorController.getYButtonReleased();
  }

  public boolean controllerLeftBumper() {
    return operatorController.getLeftBumper();
  }

  public boolean controllerLeftBumperPressed() {
    return operatorController.getLeftBumperPressed();
  }

  public boolean controllerLeftBumperReleased() {
    return operatorController.getLeftBumperReleased();
  }

  public boolean controllerRightBumper() {
    return operatorController.getRightBumper();
  }

  public boolean controllerRightBumperPressed() {
    return operatorController.getRightBumperPressed();
  }

  public boolean controllerRightBumperReleased() {
    return operatorController.getRightBumperReleased();
  }

  public double controllerLeftTrigger() {
    return operatorController.getLeftTriggerAxis();
  }

  public double controllerRightTrigger() {
    return operatorController.getRightTriggerAxis();
  }

  public int controllerPOV() {
    return operatorController.getPOV();
  }
}
