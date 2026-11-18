from django.urls import path

from . import views

app_name = "control"

urlpatterns = [
    path("", views.control_page, name="control"),
    path("archive/", views.archive_page, name="archive"),
    path("archive/<int:pk>/", views.photo_detail, name="photo_detail"),
    path("api/state/", views.api_state, name="api_state"),
    path("api/commands/", views.api_submit, name="api_submit"),
    path("api/stop/", views.api_stop, name="api_stop"),
    path("api/motors/", views.api_motors, name="api_motors"),
    path("api/interpret/", views.api_interpret, name="api_interpret"),
    path("api/photos/", views.api_photos, name="api_photos"),
]
