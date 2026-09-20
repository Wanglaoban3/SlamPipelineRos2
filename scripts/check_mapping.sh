#!/usr/bin/env bash
LOG=/root/data/mapping_run.log
grep -E "construct solver|flush_keyframe|loop detection|loop found|loop not found|pose graph optimization|nodes:|map saved|failed" $LOG 2>/dev/null | head -25
ls -la /root/data/globalmap.pcd 2>/dev/null || echo NO_MAP_YET
