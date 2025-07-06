import typer

test_app = typer.Typer()


class HydrusTestManager:
    def __init__(self):
        pass

    @staticmethod
    def run_tests():
        """Run all tests in the Hydrus software stack."""
        typer.echo("Running all tests in the Hydrus software stack...")

    @staticmethod
    def run_unit_tests():
        """Run unit tests."""
        typer.echo("Running unit tests...")

    @staticmethod
    def run_integration_tests():
        """Run integration tests."""
        typer.echo("Running integration tests...")
