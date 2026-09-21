---
title: Tags
parent: Journal
nav_order: 1
permalink: /journal/tags/
---

# Articles par tag
{: .no_toc }

{% assign tags = site.tags | sort %}
{% for tag in tags %}
<h2 id="{{ tag[0] | slugify }}">{{ tag[0] }} <span class="fs-3 text-grey-dk-000">({{ tag[1].size }})</span></h2>
<ul>
{% for post in tag[1] %}
  <li><a href="{{ post.url | relative_url }}">{{ post.title }}</a> <span class="text-grey-dk-000">– {{ post.date | date: "%d/%m/%Y" }}</span></li>
{% endfor %}
</ul>
{% endfor %}
