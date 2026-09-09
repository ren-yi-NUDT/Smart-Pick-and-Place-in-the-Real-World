from skills.base import Skill, register_skill


@register_skill("look_around")
class LookAroundSkill(Skill):
    """Scan workspace from observation positions using GLM-4.5V."""

    def execute(self, reset_pose=None, **kwargs):
        """Scan workspace and analyze scene with VLM.

        side="left" (default): cycle left arm through grasp1-4 observation poses,
            analyze the first frame with VLM (scene + spatial relations).
        side="right": move right arm to ``drawer_1_placement``, capture the
            drawer interior, ask VLM to list visible items (ignoring foam pads).

        Args:
            reset_pose: Pose name to return to after scanning.
                        None → left arm uses ``grasp1``, right arm uses ``home``.
                        Set to a string to override; pass "" to skip reset.
        """
        data = kwargs if kwargs.get("side") else (self.json_parser.get_command() or {})
        side = data.get("side", "left")

        if side == "right":
            return self.observation_pipeline.inspect_drawer(
                reset_pose="home" if reset_pose is None else reset_pose
            )
        return self.observation_pipeline.scan_workspace(
            reset_pose="grasp1" if reset_pose is None else reset_pose
        )
