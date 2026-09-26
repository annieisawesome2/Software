import csv
from collections.abc import Iterable
from pathlib import Path

from software.stats.logs.event_log import EventLog, EventType


DEFAULT_EVENTS_PATH = Path("/tmp/tbots/stats/game_events.csv")

EVALUATION_EVENT_TYPES = (
    EventType.GOAL_SCORED,
    EventType.SHOT_ON_GOAL,
    EventType.ENEMY_SHOT_ON_GOAL,
    EventType.SHOT_BLOCKED,
)


def load_events(path: str | Path = DEFAULT_EVENTS_PATH) -> list[EventLog]:
    """Load event logs from a headerless CSV file.

    :param path: Path to a CSV file written by StatsLogger.
    :return: All successfully parsed event logs.
    """
    events = []

    with open(path, newline="") as events_file:
        for row in csv.reader(events_file):
            if not row:
                continue

            event = EventLog.from_csv_row(iter(row))
            if event is not None:
                events.append(event)

    return events


def count_events(events: Iterable[EventLog], event_type: EventType) -> int:
    """Count events matching the requested type."""
    return sum(event.event_type == event_type for event in events)


def generate_evaluation_metrics(events: Iterable[EventLog]) -> dict[EventType, int]:
    """Generate basic gameplay-event counts."""
    events = list(events)
    return {
        event_type: count_events(events, event_type)
        for event_type in EVALUATION_EVENT_TYPES
    }


def main() -> None:
    """Print basic evaluation metrics from the runtime event log."""
    if not DEFAULT_EVENTS_PATH.exists():
        raise SystemExit(
            f"No event log found at {DEFAULT_EVENTS_PATH}. "
            "Run an AI-vs-AI simulation with --record_stats first."
        )

    metrics = generate_evaluation_metrics(load_events())

    print(f"Evaluation metrics from {DEFAULT_EVENTS_PATH}:")
    for event_type, count in metrics.items():
        print(f"{event_type.value}: {count}")


if __name__ == "__main__":
    main()
