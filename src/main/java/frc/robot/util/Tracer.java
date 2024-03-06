package frc.robot.util;

import org.littletonrobotics.junction.Logger;

public class Tracer {

  public static boolean enableTrace = true;

  public static void trace(String message) {
    if (!enableTrace) {
      return;
    }
    System.err.println("[TRACE] -- " + message);
    Logger.recordOutput("Auto/Trace", message);
  }
}
