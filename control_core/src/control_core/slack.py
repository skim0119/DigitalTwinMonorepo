"""Post experiment results to the Slack channel watched by broker."""

import json
import os
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def load_settings(env_file):
    if env_file.is_file():
        try:
            from dotenv import load_dotenv
        except ImportError as error:
            raise RuntimeError("Reading .env for Slack requires python-dotenv.") from error
        load_dotenv(env_file)
    token = os.environ.get("SLACK_BOT_TOKEN")
    channel = os.environ.get("SLACK_CHANNEL")
    if not token and not channel:
        return None
    if not token or not channel:
        raise RuntimeError("Set SLACK_BOT_TOKEN and SLACK_CHANNEL in the repository .env.")
    return token, channel


def post_message(settings, text):
    token, channel = settings
    request = Request(
        "https://slack.com/api/chat.postMessage",
        data=json.dumps({"channel": channel, "text": text, "mrkdwn": False}).encode(),
        headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
    )
    # shortcut: retry for up to three attempts in-process, use a durable queue if offline delivery is needed.
    for attempt in range(3):
        delay = 2 ** attempt
        try:
            with urlopen(request, timeout=5) as response:
                result = json.load(response)
        except HTTPError as error:
            failure = f"HTTP {error.code}"
            if error.code == 429:
                retry_after = error.headers.get("Retry-After", "30")
                if not retry_after.isdecimal() or int(retry_after) > 30:
                    break
                delay = int(retry_after)
            elif error.code < 500:
                break
        except OSError as error:
            failure = type(error).__name__
        except ValueError:
            failure = "invalid JSON response"
        else:
            if isinstance(result, dict) and result.get("ok") is True:
                return
            failure = "Slack rejected the message"
            break
        if attempt < 2:
            time.sleep(delay)
    raise RuntimeError(f"Slack delivery could not be confirmed ({failure}).")
