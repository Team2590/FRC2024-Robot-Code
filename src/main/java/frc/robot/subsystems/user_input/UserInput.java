package frc.robot.subsystems.user_input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.util.BandedJoystick;
import frc.robot.util.NemesisSubsystem;
import frc.robot.util.Smoother;

/**
 * Handles input from 2 joysticks and a button panel. The button panel is numbered 1 to 18, numbered
 * as if reading English.
 *
 * @author Elan Ronen
 * @see <a
 *     href="https://www.digitalcombatsimulator.com/upload/iblock/3c2/Template-T16000m-Hotas.jpg">Joystick
 *     Button Map</a>\
 */
public class UserInput extends NemesisSubsystem implements RobotMap {

  private static UserInput instance;

  public static UserInput getInstance() {
    return instance == null ? instance = new UserInput() : instance;
  }

  private final BandedJoystick leftJoystick;
  private final BandedJoystick rightJoystick;
  private final Smoother.Wrapper leftXInput;
  private final Smoother.Wrapper leftYInput;
  private final Smoother.Wrapper rightXInput;
  private final Joystick operatorPanel;
  private final XboxController operatorController;

  private UserInput() {

    leftJoystick = new BandedJoystick(LEFT_JOYSTICK, 0.1, 0.1);
    rightJoystick = new BandedJoystick(RIGHT_JOYSTICK, 0.1, 0.1);
    leftXInput = Smoother.wrap(2, leftJoystick::getXBanded);
    leftYInput = Smoother.wrap(2, leftJoystick::getYBanded);
    rightXInput = Smoother.wrap(1, rightJoystick::getXBanded);
    operatorPanel = new Joystick(3);
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

  public int leftJoystickPOV() {
    return leftJoystick.getPOV();
  }

  public boolean leftJoystickTrigger() {
    return leftJoystick.getTrigger();
  }

  public boolean leftJoystickTriggerPressed() {
    return leftJoystick.getTriggerPressed();
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

  public int rightJoystickPOV() {
    return rightJoystick.getPOV();
  }

  public boolean rightJoystickTrigger() {
    return rightJoystick.getTrigger();
  }

  public boolean rightJoystickTriggerPressed() {
    return rightJoystick.getTriggerPressed();
  }

  // OPERATOR PANEL

  public boolean operatorPanelButton(int button) {
    return operatorPanel.getRawButton(button);
  }

  public boolean operatorPanelButtonPressed(int button) {
    return operatorPanel.getRawButtonPressed(button);
  }

  public boolean operatorPanelButtonReleased(int button) {
    return operatorPanel.getRawButtonReleased(button);
  }

  public boolean leftJoystickButtonReleased(int button) {
    return leftJoystick.getRawButtonReleased(button);
  }

  public boolean rightJoystickButtonReleased(int button) {
    return rightJoystick.getRawButtonReleased(button);
  }

  // OPERATOR CONTROLLER
  public boolean XbuttonPressed() {
    return operatorController.getXButtonPressed();
  }

  public boolean YbuttonPressed() {
    return operatorController.getYButtonPressed();
  }

  public boolean BbuttonPressed() {
    return operatorController.getBButtonPressed();
  }

  public boolean AbuttonPressed() {
    return operatorController.getAButtonPressed();
  }

  public boolean controllerLeftTriggerPressed() {
    return operatorController.getLeftBumperPressed();
  }

  public boolean controllerRightTriggerPressed() {
    return operatorController.getRightBumperPressed();
  }
}
