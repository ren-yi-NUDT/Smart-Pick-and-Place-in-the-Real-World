from skills.base import Skill, register_skill


@register_skill("capture_at_handover")
class CaptureAtHandoverSkill(Skill):
    """Move to handover viewing pose, capture image, analyze with VLM."""

    def execute(self, reset_pose="grasp1", **kwargs):
        """Move to handover viewing pose, capture image, analyze with VLM.

        Args:
            reset_pose: Pose name to return to after capture.
                        Set to None to skip reset (for chained calls).
        """
        return self.observation_pipeline.capture_at_handover(
            reset_pose=reset_pose
        )
