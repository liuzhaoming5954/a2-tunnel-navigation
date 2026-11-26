"""验证站端双跳遥测模板。只使用临时配置，不建立现场连接。"""

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

    def effective_ssh(self, command):
        result = subprocess.run([command[0], "-G", *command[1:]], text=True,
                                capture_output=True, check=True, timeout=3)
        return dict(line.split(" ", 1) for line in result.stdout.splitlines())

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


if __name__ == "__main__":
    unittest.main()
