import logging
from typing import Annotated

import typer

logger = logging.getLogger(__name__)

tg2_app = typer.Typer(help="Placeholder CLI for TG2 hardware integration (not implemented yet).")


@tg2_app.command()
def info(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Prints a stub info message for TG2."""
    logger.info("TG2 at %s: hardware integration not implemented.", ip)
    typer.echo(f"TG2 at {ip}: hardware integration not implemented.")


@tg2_app.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Stub home command."""
    typer.echo(f"Would home TG2 at {ip} (stub).")


if __name__ == "__main__":
    tg2_app()
