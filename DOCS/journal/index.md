---
title: Journal
nav_order: 2
has_children: true
has_toc: false
permalink: /journal/
---

# Journal de bord
{: .no_toc }

Conception, essais et retours d'expérience, du plus récent au plus ancien.
[Par tag]({{ '/journal/tags/' | relative_url }}) · [Flux RSS]({{ '/feed.xml' | relative_url }})

{% for post in site.posts %}
<div class="mb-6">
  <h2 class="mb-1"><a href="{{ post.url | relative_url }}">{{ post.title }}</a></h2>
  <p class="fs-2 text-grey-dk-000 mb-1">
    {{ post.date | date: "%d/%m/%Y" }}
    {% for tag in post.tags %}<span class="label">{{ tag }}</span> {% endfor %}
  </p>
  {% if post.description %}<p class="mt-0">{{ post.description }}</p>{% endif %}
</div>
{% else %}
Aucun article pour le moment.
{% endfor %}
