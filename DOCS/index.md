---
title: Accueil
layout: home
nav_order: 1
---

# CDR 2027
{: .no_toc }

Journal de bord et documentation du robot différentiel Karibous, pour la Coupe de France de robotique 2027.

## Derniers articles

{% for post in site.posts limit: 3 %}
- [{{ post.title }}]({{ post.url | relative_url }}) – {{ post.date | date: "%d/%m/%Y" }}
{% endfor %}
