---
title:  "Programming"
permalink: /programming/
layout: archive
author_profile: true
classes: wide
---

{% assign posts = site.categories['Programming'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}