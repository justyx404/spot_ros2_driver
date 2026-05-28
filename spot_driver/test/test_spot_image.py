import importlib
import inspect
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest


class FakePublisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return "stamp"


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.logger = FakeLogger()
        self.clock = FakeClock()

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher(msg_type, topic, qos)
        self.publishers.append(publisher)
        return publisher

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock


class FakeImageClient:
    def __init__(self, sources=None, responses=None):
        self.sources = sources or []
        self.responses = responses or []
        self.requests = []

    def list_image_sources(self):
        return [SimpleNamespace(name=source) for source in self.sources]

    def get_image(self, requests):
        self.requests.extend(requests)
        return self.responses


@pytest.fixture()
def spot_image_module(monkeypatch):
    build_requests = []

    class BosdynImage:
        FORMAT_JPEG = 1
        FORMAT_RAW = 2
        PIXEL_FORMAT_GREYSCALE_U8 = 10
        PIXEL_FORMAT_RGB_U8 = 11

    image_pb2 = types.ModuleType("bosdyn.api.image_pb2")
    image_pb2.Image = BosdynImage

    def build_image_request(
        image_source_name,
        quality_percent=75,
        image_format=None,
        pixel_format=None,
        fallback_formats=None,
    ):
        request = {
            "image_source_name": image_source_name,
            "quality_percent": quality_percent,
            "image_format": image_format,
            "pixel_format": pixel_format,
            "fallback_formats": fallback_formats,
        }
        build_requests.append(request)
        return request

    bosdyn = types.ModuleType("bosdyn")
    bosdyn_api = types.ModuleType("bosdyn.api")
    bosdyn_api.image_pb2 = image_pb2
    bosdyn_client = types.ModuleType("bosdyn.client")
    bosdyn_client_frame_helpers = types.ModuleType("bosdyn.client.frame_helpers")
    bosdyn_client_frame_helpers.BODY_FRAME_NAME = "body"
    bosdyn_client_frame_helpers.get_a_tform_b = lambda *_args, **_kwargs: "body_tform_camera"
    bosdyn_client_image = types.ModuleType("bosdyn.client.image")
    bosdyn_client_image.ImageClient = type("ImageClient", (), {})
    bosdyn_client_image.build_image_request = build_image_request
    bosdyn_client_math_helpers = types.ModuleType("bosdyn.client.math_helpers")
    bosdyn_client_math_helpers.SE3Pose = type("SE3Pose", (), {})

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})

    class Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class CameraInfo:
        def __init__(self):
            self.header = Header()
            self.height = 0
            self.width = 0
            self.distortion_model = ""
            self.d = []
            self.k = []
            self.r = []
            self.p = []

    class CompressedImage:
        def __init__(self):
            self.header = Header()
            self.format = ""
            self.data = b""

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = CameraInfo
    sensor_msgs_msg.CompressedImage = CompressedImage

    modules = {
        "bosdyn": bosdyn,
        "bosdyn.api": bosdyn_api,
        "bosdyn.api.image_pb2": image_pb2,
        "bosdyn.client": bosdyn_client,
        "bosdyn.client.frame_helpers": bosdyn_client_frame_helpers,
        "bosdyn.client.image": bosdyn_client_image,
        "bosdyn.client.math_helpers": bosdyn_client_math_helpers,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    package_root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(package_root))
    sys.modules.pop("spot_driver.spot_image", None)
    module = importlib.import_module("spot_driver.spot_image")
    return module, build_requests


def make_image_response(spot_image_module, source_name="hand_color_image", data=b"jpeg-bytes", rows=1080, cols=1920):
    intrinsics = SimpleNamespace(
        focal_length=SimpleNamespace(x=100.0, y=101.0),
        principal_point=SimpleNamespace(x=50.0, y=51.0),
    )
    source = SimpleNamespace(
        name=source_name,
        rows=rows,
        cols=cols,
        pinhole=SimpleNamespace(intrinsics=intrinsics),
    )
    image = SimpleNamespace(
        data=data,
        format=spot_image_module.image_pb2.Image.FORMAT_JPEG,
        pixel_format=spot_image_module.image_pb2.Image.PIXEL_FORMAT_RGB_U8,
        rows=rows,
        cols=cols,
    )
    shot = SimpleNamespace(
        frame_name_image_sensor=f"{source_name}_frame",
        image=image,
        transforms_snapshot=object(),
    )
    return SimpleNamespace(source=source, shot=shot)


def test_compressed_publishers_are_only_image_transport(spot_image_module):
    module, _build_requests = spot_image_module
    node = FakeNode()
    image_client = FakeImageClient(sources=["hand_color_image"])

    module.SpotImagePublisher(
        node,
        image_client=image_client,
        include_gripper_camera=True,
        image_qos="sensor_qos",
    )

    topics = {publisher.topic for publisher in node.publishers}
    assert "camera/frontleft_fisheye/image/compressed" in topics
    assert "camera/frontright_fisheye/image/compressed" in topics
    assert "camera/hand_color/image/compressed" in topics
    assert "camera/frontleft_fisheye/image" not in topics
    assert "camera/frontright_fisheye/image" not in topics
    assert "camera/hand_color/image" not in topics
    assert all(publisher.qos == "sensor_qos" for publisher in node.publishers)


def test_removed_image_options_stay_removed(spot_image_module):
    module, _build_requests = spot_image_module

    signature = inspect.signature(module.SpotImagePublisher)

    assert "publish_raw_images" not in signature.parameters
    assert "gripper_camera_resize_ratio" not in signature.parameters


def test_source_settings_only_include_sdk_request_pixel_formats(spot_image_module):
    module, _build_requests = spot_image_module

    assert not hasattr(module.SpotImagePublisher, "SOURCE_SETTINGS")
    assert module.SpotImagePublisher.SOURCE_PIXEL_FORMATS == {
        "frontleft_fisheye_image": module.image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
        "frontright_fisheye_image": module.image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
        "hand_color_image": module.image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    }


def test_gripper_source_is_the_only_dynamic_tf_source(spot_image_module):
    module, _build_requests = spot_image_module

    assert not hasattr(module.SpotImagePublisher, "DYNAMIC_IMAGE_SOURCES")
    assert module.SpotImagePublisher.is_static_source("frontleft_fisheye_image")
    assert module.SpotImagePublisher.is_static_source("frontright_fisheye_image")
    assert not module.SpotImagePublisher.is_static_source(module.SpotImagePublisher.GRIPPER_IMAGE_SOURCE)


def test_publish_selected_source_as_compressed_jpeg(spot_image_module):
    module, build_requests = spot_image_module
    response = make_image_response(module, data=b"spot-jpeg")
    image_client = FakeImageClient(sources=["hand_color_image"], responses=[response])
    node = FakeNode()
    publisher = module.SpotImagePublisher(
        node,
        image_client=image_client,
        include_gripper_camera=True,
        jpeg_quality=80,
    )

    publisher.publish_image_and_info(image_client, sources=["hand_color_image"])

    assert build_requests == [
        {
            "image_source_name": "hand_color_image",
            "quality_percent": 80,
            "image_format": module.image_pb2.Image.FORMAT_JPEG,
            "pixel_format": module.image_pb2.Image.PIXEL_FORMAT_RGB_U8,
            "fallback_formats": None,
        }
    ]

    compressed_pub = next(pub for pub in node.publishers if pub.topic == "camera/hand_color/image/compressed")
    assert len(compressed_pub.published) == 1
    compressed_msg = compressed_pub.published[0]
    assert compressed_msg.header.stamp == "stamp"
    assert compressed_msg.header.frame_id == "hand_color_image_frame"
    assert compressed_msg.format == "jpeg"
    assert compressed_msg.data == b"spot-jpeg"

    info_pub = next(pub for pub in node.publishers if pub.topic == "camera/hand_color/camera_info")
    assert len(info_pub.published) == 1
    assert info_pub.published[0].height == 1080
    assert info_pub.published[0].width == 1920


def test_publish_selected_source_only_requests_that_source(spot_image_module):
    module, build_requests = spot_image_module
    response = make_image_response(module, source_name="frontleft_fisheye_image")
    image_client = FakeImageClient(responses=[response])
    node = FakeNode()
    publisher = module.SpotImagePublisher(node)

    publisher.publish_image_and_info(image_client, sources=["frontleft_fisheye_image"])

    assert [request["image_source_name"] for request in build_requests] == ["frontleft_fisheye_image"]
    frontleft_pub = next(pub for pub in node.publishers if pub.topic == "camera/frontleft_fisheye/image/compressed")
    frontright_pub = next(pub for pub in node.publishers if pub.topic == "camera/frontright_fisheye/image/compressed")
    assert len(frontleft_pub.published) == 1
    assert frontright_pub.published == []
