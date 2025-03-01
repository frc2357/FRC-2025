package frc.robot.commands.auto;

public class CR3RedStation4Peice extends AutoBase {

  public CR3RedStation4Peice() {
    // do NOT start with this one. The other auto is the one to tune first.
    super("Cage R3 | 2 P | Red Station", "CR3ToBranchF");
    makeAutoFromSegments(
      "branchFToRedS",
      "RedSToBranchE",
      "branchEToRedS",
      "RedSToBranchD",
      "branchDToRedS",
      "RedSToBranchC",
      "branchCToRedS"
    );
  }
}
