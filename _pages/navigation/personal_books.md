---
title:  "Personal / 독서"
permalink: /personal/books/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.tags['독서'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}