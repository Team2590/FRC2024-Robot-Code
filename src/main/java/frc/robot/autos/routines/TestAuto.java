package frc.robot.autos.routines;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TrajectoryFollowerCommand;


public class TestAuto extends SequentialCommandGroup {

    TestAuto(Drive drive){
        addRequirements(drive);
    }

}