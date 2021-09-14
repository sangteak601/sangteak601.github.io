---
title:  "Theory"
permalink: /theory/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.categories['Theory'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}