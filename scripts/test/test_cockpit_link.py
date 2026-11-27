"""验证站端双跳连接模板。只使用临时配置，不建立现场连接。"""

from configparser import ConfigParser
from pathlib import Path
import shlex
import shutil
import subprocess
from tempfile import TemporaryDirectory
import unittest


TEMPLATES = Path(__file__).resolve().parents[2] / "deploy" / "templates"


@unittest.skipUnless(shutil.which("ssh"), "需要 OpenSSH 客户端")
class CockpitLinkTest(unittest.TestCase):
    def setUp(self):
        work = TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.root = Path(work.name)
        # 模拟 systemd 的用户目录和运行目录，所有主机文件均指向临时位置。
        self.config = self.root / ".config/a2-cockpit/ssh_config"
        self.config.parent.mkdir(parents=True)
        template = (TEMPLATES / "a2-cockpit-ssh.conf.example").read_text()
        self.config.write_text(template.replace("~/.config/a2-cockpit", str(self.config.parent)))

    def service_command(self, template):
        unit = ConfigParser(interpolation=None)
        unit.read(TEMPLATES / template)
        command = unit["Service"]["ExecStart"].replace("%h", str(self.root)).replace("%t", str(self.root))
        return shlex.split(command)

    def ssh_options(self, command):
        result = subprocess.run([command[0], "-G", *command[1:]], text=True,
                                capture_output=True, check=True, timeout=3)
        return [line.split(" ", 1) for line in result.stdout.splitlines()]

    def effective_ssh(self, command):
        return dict(self.ssh_options(command))

    def test_master_and_jump_use_fixed_identities_and_pinned_hosts(self):
        link = self.effective_ssh(self.service_command("a2-cockpit-link.service"))
        jump = self.effective_ssh(shlex.split(link["proxycommand"]))
        self.assertEqual(jump["host"], "a2-cockpit-pc3")
        for config, hop in ((link, "pc2"), (jump, "pc3")):
            with self.subTest(hop=hop):
                self.assertEqual(config["batchmode"], "yes")
                self.assertEqual(config["identitiesonly"], "yes")
                self.assertEqual(config["stricthostkeychecking"], "true")
                self.assertEqual(config["clearallforwardings"], "yes")
                self.assertEqual(config["identityfile"], str(self.config.parent / (hop + "_ed25519")))
                self.assertEqual(config["userknownhostsfile"], str(self.config.parent / "known_hosts"))
                self.assertEqual(config["requesttty"], "false")
        self.assertEqual(link["controlmaster"], "true")
        self.assertEqual(jump["controlmaster"], "false")
        self.assertNotIn("controlpath", jump)
        self.assertNotEqual(link["serveraliveinterval"], "0")
        self.assertNotEqual(link["serveralivecountmax"], "0")

    def test_stream_without_master_fails_without_another_authentication(self):
        stream = self.service_command("a2-cockpit-relay@.service")
        effective = self.effective_ssh(stream)
        self.assertEqual(effective["proxycommand"], "/bin/false")
        self.assertEqual(effective["controlmaster"], "false")
        # Master 路径不存在。OpenSSH 只能走本地 /bin/false，立即断开该 HTTP 连接。
        result = subprocess.run(stream, stdin=subprocess.DEVNULL, capture_output=True, timeout=3)
        self.assertEqual(result.returncode, 255)
        self.assertEqual(result.stdout, b"")

    def test_control_and_keyboard_use_independent_loopback_forwards(self):
        readonly = self.effective_ssh(self.service_command("a2-cockpit-link.service"))
        for kind, port in (("control", 18777), ("keyboard", 18778)):
            with self.subTest(kind=kind):
                command = self.service_command(f"a2-cockpit-{kind}-link.service")
                options = self.ssh_options(command)
                configuration = dict(options)
                self.assertEqual(configuration["user"], "a2-cockpit-control")
                self.assertEqual(configuration["identityfile"], str(self.config.parent / "pc2_control_ed25519"))
                self.assertEqual(configuration["proxycommand"], readonly["proxycommand"])
                self.assertEqual(configuration["batchmode"], "yes")
                self.assertEqual(configuration["stricthostkeychecking"], "true")
                self.assertEqual(configuration["exitonforwardfailure"], "yes")
                self.assertEqual(configuration["controlmaster"], "false")
                self.assertNotIn("controlpath", configuration)
                self.assertEqual(configuration["sessiontype"], "none")
                self.assertEqual(configuration["requesttty"], "false")
                forwarding = [value.replace("[", "").replace("]", "").split()
                              for key, value in options if key == "localforward"]
                self.assertEqual(forwarding, [[f"127.0.0.1:{port}", f"127.0.0.1:{port}"]])
                self.assertNotIn("remoteforward", configuration)

    def test_pc2_forwarding_account_has_no_session_or_other_forwarding(self):
        config = (TEMPLATES / "a2-cockpit-control-sshd.conf.example").read_text().splitlines()
        rules = [shlex.split(line, comments=True) for line in config]
        rules = [fields for fields in rules if fields]
        self.assertEqual(rules[0], ["Match", "User", "a2-cockpit-control"])
        self.assertEqual(rules[-1], ["Match", "all"])
        settings = {name: args for name, *args in rules[1:-1]}
        self.assertEqual(settings["AllowTcpForwarding"], ["local"])
        self.assertEqual(settings["PermitOpen"], ["127.0.0.1:18777", "127.0.0.1:18778"])
        self.assertEqual(settings["PermitListen"], ["none"])
        self.assertEqual(settings["AllowStreamLocalForwarding"], ["no"])
        self.assertEqual(settings["AuthenticationMethods"], ["publickey"])
        self.assertEqual(settings["MaxSessions"], ["0"])

    def test_transport_retries_and_pc2_gateway_can_be_enabled_without_sender(self):
        for kind in ("control", "keyboard"):
            with self.subTest(kind=kind):
                unit = ConfigParser(interpolation=None)
                unit.read(TEMPLATES / f"a2-cockpit-{kind}-link.service")
                self.assertEqual(unit["Unit"]["StartLimitIntervalSec"], "0")
                self.assertEqual(unit["Service"]["Restart"], "always")
                self.assertEqual(unit["Install"]["WantedBy"], "default.target")
                for option in ("Wants", "Requires", "PartOf", "BindsTo"):
                    self.assertEqual(unit["Unit"].get(option, ""), "")
        gateway = ConfigParser(interpolation=None)
        gateway.read(TEMPLATES / "a2-cockpit-manual-gateway.service")
        self.assertEqual(gateway["Install"]["WantedBy"], "multi-user.target")
        for option in ("Wants", "Requires"):
            self.assertNotIn("a2-autonomy.service", gateway["Unit"].get(option, "").split())


if __name__ == "__main__":
    unittest.main()
