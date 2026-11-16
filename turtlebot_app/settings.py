"""Django settings for the TurtleBot web app.

Everything machine-specific comes from environment variables (or a `.env` file next to manage.py, see .env.example):
  ROBOT_BACKEND        "agent" (real robot via robot_agent/agent.py) or "sim" (built-in simulator). Default: sim
  ROBOT_AGENT_URL      e.g. http://192.168.137.140:8765
  ROBOT_AGENT_TOKEN    same secret as AGENT_TOKEN on the robot
  DJANGO_SECRET_KEY    optional; otherwise one is generated once into .secret_key (git-ignored)
  DJANGO_DEBUG         1/0, default 1 (this is a local lab tool served by runserver)
  DJANGO_ALLOWED_HOSTS comma-separated, default localhost,127.0.0.1
  ARCHIVE_DIR          where photos are stored, default ./archive
"""

import os
import secrets
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent


def _load_dotenv(path):
    """Minimal .env reader: KEY=VALUE lines; real environment variables win."""
    if not path.exists():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


_load_dotenv(BASE_DIR / ".env")


def _secret_key():
    key = os.environ.get("DJANGO_SECRET_KEY")
    if key:
        return key
    path = BASE_DIR / ".secret_key"
    if not path.exists():
        path.write_text(secrets.token_urlsafe(50), encoding="utf-8")
    return path.read_text(encoding="utf-8").strip()


SECRET_KEY = _secret_key()
DEBUG = os.environ.get("DJANGO_DEBUG", "1") == "1"
ALLOWED_HOSTS = [
    h.strip()
    for h in os.environ.get("DJANGO_ALLOWED_HOSTS", "localhost,127.0.0.1").split(",")
    if h.strip()
]

INSTALLED_APPS = [
    "django.contrib.admin",
    "django.contrib.auth",
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.messages",
    "django.contrib.staticfiles",
    "control",
]

MIDDLEWARE = [
    "django.middleware.security.SecurityMiddleware",
    "django.contrib.sessions.middleware.SessionMiddleware",
    "django.middleware.common.CommonMiddleware",
    "django.middleware.csrf.CsrfViewMiddleware",
    "django.contrib.auth.middleware.AuthenticationMiddleware",
    "django.contrib.messages.middleware.MessageMiddleware",
    "django.middleware.clickjacking.XFrameOptionsMiddleware",
]

ROOT_URLCONF = "turtlebot_app.urls"

TEMPLATES = [
    {
        "BACKEND": "django.template.backends.django.DjangoTemplates",
        "DIRS": [],
        "APP_DIRS": True,
        "OPTIONS": {
            "context_processors": [
                "django.template.context_processors.request",
                "django.contrib.auth.context_processors.auth",
                "django.contrib.messages.context_processors.messages",
            ],
        },
    },
]

WSGI_APPLICATION = "turtlebot_app.wsgi.application"

DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.sqlite3",
        "NAME": BASE_DIR / "db.sqlite3",
        # The command runner writes from a background thread while pages read; wait instead of failing.
        "OPTIONS": {"timeout": 20},
    }
}

AUTH_PASSWORD_VALIDATORS = [
    {"NAME": "django.contrib.auth.password_validation.UserAttributeSimilarityValidator"},
    {"NAME": "django.contrib.auth.password_validation.MinimumLengthValidator"},
    {"NAME": "django.contrib.auth.password_validation.CommonPasswordValidator"},
    {"NAME": "django.contrib.auth.password_validation.NumericPasswordValidator"},
]

LANGUAGE_CODE = "en-us"
TIME_ZONE = "America/Phoenix"  # the lab is in Tucson; Arizona has no daylight saving time
USE_I18N = True
USE_TZ = True

STATIC_URL = "static/"
MEDIA_URL = "media/"
MEDIA_ROOT = Path(os.environ.get("ARCHIVE_DIR", BASE_DIR / "archive"))

DEFAULT_AUTO_FIELD = "django.db.models.BigAutoField"

# ---- robot

ROBOT_BACKEND = os.environ.get("ROBOT_BACKEND", "sim")
ROBOT_AGENT_URL = os.environ.get("ROBOT_AGENT_URL", "http://192.168.137.140:8765")
ROBOT_AGENT_TOKEN = os.environ.get("ROBOT_AGENT_TOKEN", "")
# Seconds the simulator spends per motion (0 = instant; tests set 0).
ROBOT_SIM_DELAY = float(os.environ.get("ROBOT_SIM_DELAY", "1.0"))
# Dotted path of the class that turns typed text into commands (swap in an LLM-backed one later).
COMMAND_INTERPRETER = os.environ.get(
    "COMMAND_INTERPRETER", "control.interpreter.RuleBasedInterpreter"
)

LOGGING = {
    "version": 1,
    "disable_existing_loggers": False,
    "handlers": {"console": {"class": "logging.StreamHandler"}},
    "loggers": {"control": {"handlers": ["console"], "level": "INFO"}},
}
