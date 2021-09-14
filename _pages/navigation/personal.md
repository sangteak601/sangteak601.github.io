---
title:  "Personal"
permalink: /personal/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.categories['personal'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}