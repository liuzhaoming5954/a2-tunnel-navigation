#!/usr/bin/env python3
"""在 PC2 本地查看、签发或撤销遥控许可。"""

import argparse
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1] / 'src'
sys.path[:0] = [str(root / 'a2_safety'), str(root / 'a2_gateway')]
from manual_grant import DIRECTIONS, MAX_SECONDS, grant_status, identity, issue, revoke
from manual_authority import neutral_epoch, read_owner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='operation', required=True)
    commands.add_parser('status', help='读取当前授权和代号')
    grant = commands.add_parser('grant', help='根据刚读取的代号签发一次许可')
    grant.add_argument('--expect-serial', type=int, required=True, help='本机 status 返回的 serial')
    mode = grant.add_mutually_exclusive_group(required=True)
    mode.add_argument('--seconds', type=int, help=f'单次会话有效秒数，范围 1 至 {MAX_SECONDS}')
    mode.add_argument('--repeated-actions', action='store_true',
                      help='启用后可重复点动，每段动作仍用独立的硬时限')
    mode.add_argument('--continuous-hold', action='store_true',
                      help='仅供已启用对应模式的接收器，按同一次实体方向键持续授权')
    grant.add_argument('--direction', choices=sorted(DIRECTIONS), action='append', required=True,
                       help='本轮允许的机身方向，每个方向单独指定')
    grant.add_argument('--site-front-confirmed', action='store_true',
                       help='现场已确认扫掠带净空，仅限 5 秒、一个 forward 方向')
    commands.add_parser('revoke', help='立即撤销本地许可')
    args = parser.parse_args()
    try:
        if args.operation == 'status':
            # 锁存时也展示手柄回中与授权状态，便于独立核对各项条件。
            boot_id, receiver_id = identity()
            reply = grant_status(boot_id, receiver_id)
            owner = read_owner(boot_id, receiver_id)
            reply.update(boot_id=boot_id, receiver_id=receiver_id, owner=owner,
                         handset_neutral=neutral_epoch(owner) is not None)
        elif args.operation == 'revoke':
            reply = revoke()
        else:
            reply = issue(args.expect_serial, args.seconds, args.direction,
                          site_front_confirmed=args.site_front_confirmed, repeated_actions=args.repeated_actions,
                          continuous_hold=args.continuous_hold)
    except (OSError, ValueError, KeyError, TypeError, OverflowError, RecursionError) as error:
        reply = {'ok': False, 'reason': 'local_grant_unavailable', 'detail': str(error)}
    print(json.dumps(reply, ensure_ascii=False, allow_nan=False))
    return 0 if reply.get('ok', True) else 1


if __name__ == '__main__':
    sys.exit(main())
