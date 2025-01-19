package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    // H! I've tested this on Shuffleboard 2024 and 2025, it only works on 2024
    // This comment is likely outdated, as it was from when I was using 2024 WPI
    private SendableChooser<Command> chooser;
    @SuppressWarnings("unused")
    private ShuffleboardTab tab;

    public AutoSelector(ShuffleboardTab tab) {
        this.tab = tab;
        chooser = new SendableChooser<Command>();

        chooser.setDefaultOption("None", null);

        tab.add("AutoSelector", chooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);
    }

    public AutoSelector add(Command cmd, String name) {
        chooser.addOption(name, cmd);
        return this;
    }

    public AutoSelector add(Command cmd) {
        return add(cmd, cmd.getName());
    }

    public AutoSelector addDefault(Command cmd, String name) {
        chooser.setDefaultOption(name, cmd);
        return this;
    }

    public AutoSelector addDefault(Command cmd) {
        return addDefault(cmd, cmd.getName());
    }

    public Command get() {
        return chooser.getSelected();
    }
}
