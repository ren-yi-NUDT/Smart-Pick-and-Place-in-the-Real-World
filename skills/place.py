#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vision-based placement skill (thin shell).

Implementation lives in core/place_pipeline.py; pick_and_place and
fetch_from_user reuse the same pipeline through Skill.place_pipeline.
"""

from skills.base import Skill, register_skill


@register_skill("place")
class PlaceSkill(Skill):
    """Detect container, compute safe 3D target, place and verify."""

    def execute(self, **kwargs):
        return self.place_pipeline.run(**kwargs)
