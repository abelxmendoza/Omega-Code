
import pytest

@pytest.fixture(scope="session")
def setup_environment():
    # Setup code
    yield
    # Teardown code

