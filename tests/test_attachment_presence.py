import json
import os
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from aiohttp import ClientResponseError, RequestInfo
from dtps import DTPSContext
from dtps_http import RawData
from duckietown_messages.network.dtps.context import DTPSContextMsg
from duckietown_messages.simulation.hil.connection.configuration import HILConnectionConfiguration

from hil_support.attachment_presence import AttachmentPresence
from hil_support.hil import HardwareInTheLoopSide, HardwareInTheLoopSupport


class AttachmentPresenceTests(unittest.IsolatedAsyncioTestCase):
    async def report(self, robot_type, hardware, configuration, *, responses=None):
        connection = HILConnectionConfiguration(
            agent_name="map_0/vehicle_0",
            simulator=DTPSContextMsg(name="duckiematrix", urls=["http://127.0.0.1:7501/"]),
        )
        remote = MagicMock(spec=DTPSContext)
        remote.__truediv__.return_value = remote
        remote.publish.side_effect = responses
        with patch.dict(os.environ, {
            "ROBOT_TYPE": robot_type, "ROBOT_HARDWARE": hardware, "ROBOT_CONFIGURATION": configuration,
        }):
            presence = AttachmentPresence("test-robot")
        with (patch("hil_support.attachment_presence.context", new=AsyncMock(return_value=remote)),
              patch("hil_support.attachment_presence._route_address", return_value="192.0.2.10")):
            await presence.update(connection, dreamwalking=False)
            await presence.update(connection, dreamwalking=True)
            await presence.close()
        return [json.loads(call.args[0].content) for call in remote.publish.await_args_list]

    async def test_reports_use_robot_platform_identity_and_live_dreamwalking(self):
        for robot_type, platform, configuration, expected in (
            ("duckiebot", "virtual", "DB21M", "virtual"),
            ("duckiedrone", "jetson_orin_nano", "DD24", "physical"),
            ("traffic_light", "raspberry_pi", "TL21", "physical"),
            ("watchtower", "raspberry_pi_64", "WT21A", "physical"),
            ("duckiecam", "__NOTSET__", "DC21", "unknown"),
        ):
            with self.subTest(robot_type=robot_type, platform=platform):
                attach, refresh, detach = await self.report(robot_type, platform, configuration)
                self.assertEqual(attach["robot_type"], robot_type)
                self.assertEqual(attach["configuration"], configuration)
                self.assertEqual(attach["hardware"], expected)
                self.assertEqual(attach["address"], "192.0.2.10")
                if expected == "physical":
                    self.assertIs(attach["dreamwalking"], False)
                    self.assertEqual(refresh, {**attach, "dreamwalking": True})
                else:
                    self.assertNotIn("dreamwalking", attach)
                    self.assertEqual(attach, refresh)
                self.assertTrue(attach["attached"])
                self.assertFalse(detach["attached"])
                self.assertEqual(detach["instance"], attach["instance"])

    async def test_a_rejected_attachment_never_falls_back_to_a_name_only_report(self):
        rejected = ClientResponseError(
            MagicMock(spec=RequestInfo), (), status=409, message="Entity requires duckiedrone DD24.",
        )
        attach, refresh = await self.report(
            "duckiebot", "virtual", "DB21J", responses=[rejected, rejected],
        )
        self.assertEqual(attach, refresh)
        self.assertEqual(attach["configuration"], "DB21J")

    async def test_identity_is_available_before_attachment(self):
        with patch.dict(os.environ, {"ROBOT_TYPE": "duckiebot", "ROBOT_CONFIGURATION": "DB21J"}):
            presence = AttachmentPresence("test-robot")
        with patch("hil_support.attachment_presence.KVStore") as instance:
            for call in ("init", "declare", "set"):
                setattr(instance.return_value, call, AsyncMock())
            await presence.publish_identity()
        instance.return_value.set.assert_awaited_once_with(
            "hil/identity", {"robot": "test-robot", "robot_type": "duckiebot", "configuration": "DB21J"},
        )

    async def test_rejected_direct_kv_update_does_not_wire_the_passthrough(self):
        connection = HILConnectionConfiguration(
            agent_name="map_0/vehicle_0",
            simulator=DTPSContextMsg(name="duckiematrix", urls=["http://127.0.0.1:7501/"], path="/robot/"),
        )
        rejected = ClientResponseError(MagicMock(spec=RequestInfo), (), status=409, message="Robot 'other' is attached.")
        raw = RawData.json_from_native_object(connection.dict())
        remote = MagicMock(spec=DTPSContext)
        remote.__truediv__.return_value = remote
        remote.navigate.return_value = remote
        remote.publish.side_effect = [rejected, None]
        hil = HardwareInTheLoopSupport()
        hil._side = HardwareInTheLoopSide.DESTINATION
        hil._passthrough = MagicMock(set_destination=AsyncMock())
        with (patch("hil_support.hil.context", new=AsyncMock(return_value=remote)),
              patch.dict(os.environ, {
                  "VEHICLE_NAME": "test-robot", "ROBOT_TYPE": "duckiebot", "ROBOT_CONFIGURATION": "DB21J",
              })):
            with self.assertLogs("hil-support", level="WARNING") as messages:
                await hil._on_hil_connection_cfg_update(raw)
            self.assertIn("Robot 'other' is attached.", messages.output[0])
            hil._passthrough.set_destination.assert_not_called()
            remote.aclose.assert_awaited_once()
            self.assertIsNone(hil._hil_connection)
            await hil._on_hil_connection_cfg_update(raw)
            hil._passthrough.set_destination.assert_awaited_once_with(remote, ["map_0/vehicle_0"])
            await hil._on_hil_connection_cfg_update(RawData.json_from_native_object(None))
            hil._passthrough.set_destination.assert_awaited_with(None, [])
        self.assertEqual(remote.publish.await_count, 2, "Detach does not require an Engine check.")
        self.assertEqual([call.args for call in remote.__truediv__.call_args_list], [("hil",), ("reserve",)] * 2)
        self.assertEqual(json.loads(remote.publish.await_args.args[0].content), {
            "entity": "map_0/vehicle_0", "robot": "test-robot", "robot_type": "duckiebot", "configuration": "DB21J",
        })
