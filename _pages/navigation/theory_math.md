---
title:  "Theory / Mathematics"
permalink: /theory/mathematics/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.tags['mathematics'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}