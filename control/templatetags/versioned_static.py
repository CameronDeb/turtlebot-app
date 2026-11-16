"""{% vstatic 'control/app.js' %} -> /static/control/app.js?v=<mtime>, so browsers pick up edited files at once."""

import os

from django import template
from django.contrib.staticfiles import finders
from django.templatetags.static import static

register = template.Library()


@register.simple_tag
def vstatic(path):
    url = static(path)
    found = finders.find(path)
    if found:
        url += f"?v={int(os.path.getmtime(found))}"
    return url
