"""Repository content policy: markdown allowlist, comment hygiene, reachability, help parity."""
import os
import re
import subprocess

import pytest

from conftest import REPO

MD_ALLOWLIST = {
    'README.md',
    'docs/architecture.md',
    'docs/environment_setup.md',
    'docs/development_workflow.md',
    'docs/operation_and_deployment.md',
    'docs/troubleshooting_and_reset.md',
    'docs/external_resources.md',
}
SOURCE_EXT = ('.py', '.sh', '.cpp', '.hpp', '.h', '.yaml', '.yml', '.xml', '.urdf', '.srdf',
              '.sdf', '.txt', '.cfg', '.toml')
# MoveItConfigsBuilder loads these by file name from ur5_moveit_config/config/.
MOVEIT_CONVENTION = {'kinematics.yaml', 'joint_limits.yaml', 'pilz_cartesian_limits.yaml',
                     'moveit_controllers.yaml', 'moveit.rviz', 'initial_positions.yaml',
                     'sensors_3d.yaml', 'ompl_planning.yaml'}
SELF = os.path.relpath(os.path.abspath(__file__), REPO)


def tracked():
    try:
        out = subprocess.run(['git', '-C', REPO, 'ls-files', '--cached', '--others', '--exclude-standard'],
                             capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        pytest.skip('not a git checkout')
    return [p for p in out.splitlines() if os.path.isfile(os.path.join(REPO, p))]


def text(path):
    try:
        with open(os.path.join(REPO, path), encoding='utf-8') as f:
            return f.read()
    except (UnicodeDecodeError, OSError):
        return None


def sources():
    for p in tracked():
        if p == SELF or p.startswith('test/') and p.endswith('.md'):
            continue
        name = os.path.basename(p)
        if p.endswith(SOURCE_EXT) or name in ('ur5', 'run_sim', 'CMakeLists.txt') or p.endswith('.launch.py'):
            t = text(p)
            if t is not None:
                yield p, t


POSE_LITERAL = re.compile(r'-1\.5708')
JOINT_LIST_LITERAL = re.compile(
    r"shoulder_pan_joint1['\"],\s*['\"]shoulder_pan_joint['\"],\s*['\"]shoulder_lift_joint['\"],\s*"
    r"['\"]elbow_joint['\"],\s*['\"]wrist_1_joint['\"],\s*['\"]wrist_2_joint['\"],\s*['\"]wrist_3_joint")


def test_no_candle_or_joint_list_literals():
    bad = []
    for p, t in sources():
        if not p.endswith('.py'):
            continue
        if POSE_LITERAL.search(t) or JOINT_LIST_LITERAL.search(t):
            bad.append(p)
    assert bad == []


def test_markdown_allowlist():
    md = [p for p in tracked() if p.lower().endswith('.md')]
    assert sorted(set(md) - MD_ALLOWLIST) == []


def test_agent_dir_not_tracked():
    assert [p for p in tracked() if p.startswith('.agent/')] == []


DEFECT_ID = re.compile(r'\b[DRLEFHS]-\d{2,3}\b|\(R\d+\)|\bF-\d\d\b')
CHANGELOG = re.compile(r'(?i)(#|//|<!--)\s*(v\d+\b|fixed\b|restored\b|was:|previously\b|changed from\b|notice\b'
                       r'|updated\s+20\d\d|20\d\d-\d\d-\d\d)')
CODE_COMMENT = {
    '.py': re.compile(r'^\s*#\s*(import |from \S+ import|def |class |return\b|self\.\w+\s*[=(]|print\(|if .+:$|for .+:$)'),
    '.sh': re.compile(r'^\s*#\s*(if \[|fi$|for .+; do|done$|export \w+=|local \w+=|source |ros2 |python3 )'),
    '.cpp': re.compile(r'^\s*//\s*\w[\w:<>]*\s*(\(.*\)|=.*);\s*$'),
    '.yaml': re.compile(r'^\s*#\s*[a-z_]+:\s*(\S.*)?$'),
    '.xml': re.compile(r'<!--\s*<'),
}
CODE_COMMENT['.urdf'] = CODE_COMMENT['.srdf'] = CODE_COMMENT['.sdf'] = CODE_COMMENT['.xml']
CODE_COMMENT['.launch.py'] = CODE_COMMENT['.py']


def comment_rule(path):
    for ext in sorted(CODE_COMMENT, key=len, reverse=True):
        if path.endswith(ext):
            return CODE_COMMENT[ext]
    if os.path.basename(path) in ('ur5', 'run_sim'):
        return CODE_COMMENT['.sh']
    return None


def test_no_defect_ids_or_changelog_comments():
    bad = []
    for p, t in sources():
        for i, line in enumerate(t.splitlines(), 1):
            if DEFECT_ID.search(line) or CHANGELOG.search(line):
                bad.append(f'{p}:{i}: {line.strip()[:80]}')
    assert bad == []


def test_no_commented_out_code():
    bad = []
    for p, t in sources():
        rule = comment_rule(p)
        if rule is None:
            continue
        for i, line in enumerate(t.splitlines(), 1):
            if rule.search(line):
                bad.append(f'{p}:{i}: {line.strip()[:80]}')
    assert bad == []


def _referenced(path, corpus):
    name = os.path.basename(path)
    stem = name.split('.')[0]
    if name in corpus or path in corpus:
        return True
    return path.endswith('.py') and re.search(rf'\b{re.escape(stem)}\b', corpus) is not None


def test_every_file_reachable():
    """Every tracked file is a package manifest, a test, an allowlisted doc, or named elsewhere."""
    files = tracked()
    corpus_by = {p: text(p) or '' for p in files}
    unreachable = []
    for p in files:
        name = os.path.basename(p)
        if (name in ('package.xml', 'CMakeLists.txt', '.gitignore', '__init__.py', 'conftest.py')
                or p in MD_ALLOWLIST or p.startswith('test/') or '/test/' in p
                or (p.startswith('ur5_moveit_config/config/') and name in MOVEIT_CONVENTION)):
            continue
        corpus = '\n'.join(t for q, t in corpus_by.items() if q != p)
        if not _referenced(p, corpus):
            unreachable.append(p)
    assert unreachable == []


def _help_commands():
    t = text('scripts/ur5') or ''
    block = re.search(r"cat <<'HELP'\n(.*?)\nHELP", t, re.S)
    assert block, 'scripts/ur5 help block not found'
    return {m.group(1) for m in re.finditer(r'^  ur5 ([a-z][a-z_-]*)', block.group(1), re.M)}


def _readme_commands():
    t = text('README.md') or ''
    return {m.group(1) for m in re.finditer(r'^\| `ur5 ([a-z][a-z_-]*)', t, re.M)}


def test_help_matches_readme():
    assert _help_commands() == _readme_commands()


def test_dispatcher_runs_installed_entries_only():
    for p in ('scripts/ur5', 'scripts/ur5_env.sh'):
        t = text(p) or ''
        assert re.search(r'python3\s+(\S*/)?src/', t) is None, p
        assert "<<'PY'" not in t and '<<PY' not in t and "python3 - <<" not in t, p
