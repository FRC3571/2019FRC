package frc.robot.commands.auto;

import frc.robot.commands.DriveStraightDistance;
//import ca.team3571.offseason.commands.TurnWithDegrees;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PracticeAuto extends CommandGroup {

    public PracticeAuto() {

        addSequential(new DriveStraightDistance(100, 0.5));

    }

    @Override
    protected void initialize() {
        System.out.println("Executing practice autonomous");
    }
}
