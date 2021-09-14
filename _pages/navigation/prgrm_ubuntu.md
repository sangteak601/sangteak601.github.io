---
title:  "Programming / Ubuntu"
permalink: /programming/ubuntu/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.tags['ubuntu'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}