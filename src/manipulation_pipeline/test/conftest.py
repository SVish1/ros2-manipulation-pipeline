import rclpy
import pytest


@pytest.fixture(scope="session", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()
