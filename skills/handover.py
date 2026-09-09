from skills.base import Skill, register_skill


@register_skill("handover")
class HandoverSkill(Skill):
    """Release to the user through each arm's named-pose path."""

    def execute(self, **kwargs):
        side = kwargs.get("side", "left")
        return self.handover_pipeline.run(
            mode="user_release",
            side=side,
            speed=kwargs.get("speed", 15),
            release_wait=kwargs.get("release_wait", 2.0),
        )
