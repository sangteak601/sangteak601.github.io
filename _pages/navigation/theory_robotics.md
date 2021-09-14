---
title:  "Theory / Robotics"
permalink: /theory/robotics/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.tags['robotics'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}