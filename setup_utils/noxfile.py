from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import nox

DIR = Path(__file__).parent.parent.resolve()

nox.options.sessions = ["lint", "pylint", "tests"]


@nox.session
def lint(session: nox.Session) -> None:
    """
    Run the linter.
    """
    session.install("pre-commit")
    with session.chdir(DIR):
        session.run(
            "pre-commit", "run", "--all-files", "--show-diff-on-failure", *session.posargs
        )


@nox.session
def pylint(session: nox.Session) -> None:
    """
    Run PyLint.
    """
    # This needs to be installed into the package environment, and is slower
    # than a pre-commit check
    with session.chdir(DIR):
        session.install(".", "pylint")
        session.run("pylint", "coverage_control", *session.posargs)


@nox.session
def tests(session: nox.Session) -> None:
    """
    Run the unit and regular tests.
    """
    with session.chdir(DIR):
        session.install(".[test]")
        session.run("pytest", *session.posargs)

@nox.session
def build(session: nox.Session) -> None:
    """
    Build an SDist and wheel.
    """

    with session.chdir(DIR):
        build_path = DIR.joinpath("build")
        if build_path.exists():
            shutil.rmtree(build_path)

        session.install("build", "cibuildwheel")
        session.run("cibuildwheel", "--platform", "linux")
        session.run("python", "-m", "build", "--sdist")
    # session.run("python", "-m", "twine", "upload", "dist/*", "wheelhouse/*")

@nox.session
def docs(session: nox.Session) -> None:
    """
    Build the documentation.
    """
    session.run("doxygen", "doc/Doxyfile")
