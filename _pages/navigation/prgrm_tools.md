---
title:  "Programming / Tools"
permalink: /programming/tools/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.tags['tools'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}