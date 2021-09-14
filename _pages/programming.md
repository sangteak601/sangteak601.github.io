
{% assign posts = site.categories['프로그래밍'] %}
{% for post in posts %} {% include archive-single.html type=page.entries_layout %} {% endfor %}