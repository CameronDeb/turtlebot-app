from django.contrib import admin

from .models import Job, Photo, Trip, Waypoint


class WaypointInline(admin.TabularInline):
    model = Waypoint
    extra = 0
    readonly_fields = ["seq", "kind", "x", "y", "yaw", "retraced", "job", "created_at"]


@admin.register(Trip)
class TripAdmin(admin.ModelAdmin):
    list_display = ["id", "started_at", "status", "started_docked", "ended_at"]
    list_filter = ["status", "started_docked"]
    inlines = [WaypointInline]


@admin.register(Job)
class JobAdmin(admin.ModelAdmin):
    list_display = ["id", "created_at", "kind", "source", "state", "message"]
    list_filter = ["kind", "state", "source"]
    readonly_fields = ["created_at", "finished_at"]


@admin.register(Photo)
class PhotoAdmin(admin.ModelAdmin):
    list_display = ["id", "taken_at", "camera_topic", "trip", "scan_index", "note"]
    list_filter = ["camera_topic"]
    search_fields = ["note"]
    readonly_fields = ["width", "height"]
