import pytest
from fastapi import status
from pydantic import ValidationError

from src.routers.arm import ArmMoveRequest


def test_root_endpoint(client):
    response = client.get("/")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == {"message": "Interbotix Control Server is running"}


def test_ros_status_endpoint(client):
    response = client.get("/ros/status")
    assert response.status_code == status.HTTP_200_OK
    payload = response.json()
    assert payload["status"] == "running"
    assert payload["ros_launch_process_running"] is True


def test_state_machine_ready_and_start_route(client):
    ready_response = client.get("/state-machine/is-ready")
    assert ready_response.status_code == status.HTTP_200_OK
    assert ready_response.json() == {"ready": True}

    start_response = client.post(
        "/state-machine/beer-opener/start",
        json={"brand": "corona", "wait_time": 2.5},
    )
    assert start_response.status_code == status.HTTP_200_OK
    assert start_response.json() == {"message": "Beer opener started for corona"}


def test_task_route_dispatches_known_task(client):
    response = client.post("/tasks/perform/open_beer_bottle?beer_brand=sapporo")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == {"task_name": "open_beer_bottle"}

    bad_response = client.post("/tasks/perform/unknown_task?beer_brand=sapporo")
    assert bad_response.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY


def test_arm_move_request_validation_rules():
    with pytest.raises(ValidationError):
        ArmMoveRequest(
            command={"type": "set_single_joint_position", "params": {"joint_name": "waist", "position": 0.1}},
            commands=[{"type": "set_single_joint_position", "params": {"joint_name": "waist", "position": 0.1}}],
        )

    with pytest.raises(ValidationError):
        ArmMoveRequest(
            command={"type": "set_single_joint_position", "params": {"joint_name": "waist", "position": 0.1}},
            moving_time=0,
        )


def test_state_machine_stop_route(client):
    response = client.post("/state-machine/stop")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == {"message": "State machine stop requested"}
