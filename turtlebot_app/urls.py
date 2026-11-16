from django.conf import settings
from django.contrib import admin
from django.urls import include, path, re_path
from django.views.static import serve


def media(request, path):
    # The photo archive is served by Django itself: this is a local, single-user lab tool, not a public site.
    return serve(request, path, document_root=settings.MEDIA_ROOT)


urlpatterns = [
    path("admin/", admin.site.urls),
    path("", include("control.urls")),
    re_path(r"^media/(?P<path>.*)$", media),
]
